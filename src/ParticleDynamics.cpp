#include "ParticleDynamics.h"
#include "cmath"
#include "algorithm"
#include "background/SpriteSpawner.h"
#include "atomic"
#include "mutex"
#include "assert.h"

#define M_PI 3.14159265358979323846
#define PIXELS_PER_UNIT 10000.f

extern std::mutex level_lock;

// static float distance(sf::Vector2f v1, sf::Vector2f v2)
// {
//     return sqrt(pow(v1.x - v2.x, 2) + pow(v1.y - v2.y, 2));
// }

static float vlength(sf::Vector2f v)
{
    return sqrt(pow(v.x, 2) + pow(v.y, 2));
}

Particle::Particle(sf::Texture& texture, sf::Vector2f x, sf::Vector2f v, float m) : ForceSource(ForceType::Gravity, x, m)
{
    this->x_ = x;
    this->v_ = v;
    this->m_ = m;
    this->sprite_ = new SpriteObject(texture, x);
}

sf::Vector2f ForceSource::getForce(sf::Vector2f x /*pos*/, float m /*m*/)
{
    // gravitational softening length
    #define EPSILON (1.0f/100)

    float multiplier = 1.f;
    switch (this->type_)
    {
        case ForceType::AntiGravity:
        {
            multiplier = -1.f;
        }
        case ForceType::Gravity:
        {
            sf::Vector2f r = this->x_ - x;
            float d = vlength(r) / PIXELS_PER_UNIT;
            if (d > 0.0001f) r /= d; // normalize
            

            // force magnitude at distance d
            float g = G * (this->m_ / (pow(d, 2) + pow(EPSILON, 2)));

            return (r * g * m) * multiplier;
        }
        case ForceType::VectorField:
        {
            switch (vf_func)
            {
                case Wind:
                {
                    sf::Vector2f F_base = {10000, 0};
                    sf::Vector2f F_position_based = {0, (float) ((int)(x.x / 10000) % 2 == 0 ? -2000 : 2000)};
                    sf::Vector2f F_rand = {(float) (-5000 + std::rand() % 10000), (float)(-5000 + std::rand() % 5000)};
                    return F_base + F_position_based + F_rand; // placeholder
                }   
                default:
                    return {this->m_, this->m_}; // placeholder
            }
        }
        case ForceType::Constant:
        {
            return this->F_;
        }
        default:
        {
            return {0, 0};
        }
    }
}

void ForceSource::draw(sf::RenderWindow& window)
{
    if (!ParticleDynamics::draw_ff) return; 
    // if (this->type_ != ForceType::Gravity && this->type_ == ForceType::AntiGravity) return;

    sf::CircleShape c;
    c.setPosition(this->x_);
    c.setFillColor(sf::Color::White);
    c.setOrigin({5,5});
    c.setRadius(5);
    window.draw(c);
}

bool ParticleDynamics::draw_trails = false;
bool ParticleDynamics::draw_ff = false;
int ParticleDynamics::trail_seconds = 2;
bool ParticleDynamics::rk4 = true;

sf::Vector2f ParticleDynamics::getForce(Particle* self, sf::Vector2f x /*pos*/, float m /*mass*/)
{
    sf::Vector2f F = {0, 0};
    for (ForceSource* source : force_sources)
    {
        if (source == self) continue; // particle doesnt exert force on itself
        F += source->getForce(x, m);
    }

    if (self != nullptr) 
    {
        F += -self->K_ * (self->v_);
    }

    return F;
}

struct RKState {
    sf::Vector2f x;
    sf::Vector2f v;
    sf::Vector2f F;
    sf::Vector2f a;
};

void ParticleDynamics::RK4(float deltaT)
{
    RKState k1[particles.size()];
    RKState k2[particles.size()];
    RKState k3[particles.size()];
    RKState k4[particles.size()];

    // K1
    // set position and velocity
    for (size_t i = 0; i < particles.size(); i++)
    {
        Particle* p = particles[i];
        k1[i].x = p->x_;
        k1[i].v = p->v_;
    }
    // calculate forces and update acceleration
    for (size_t i = 0; i < particles.size(); i++)
    {
        Particle* p = particles[i];
        k1[i].F = getForce(p, k1[i].x, p->m_);
        k1[i].a = k1[i].F / p->m_;
    }

    // K2
    for (size_t i = 0; i < particles.size(); i++)
    {
        k2[i].x = k1[i].x + (deltaT * k1[i].v * 0.5f);
        k2[i].v = k1[i].v + (deltaT * k1[i].a * 0.5f);
    }
    for (size_t i = 0; i < particles.size(); i++)
    {
        Particle* p = particles[i];
        k2[i].F = getForce(p, k2[i].x, p->m_);
        k2[i].a = k2[i].F / p->m_;
    }

    // K3
    for (size_t i = 0; i < particles.size(); i++)
    {
        k3[i].x = k2[i].x + (deltaT * k2[i].v * 0.5f);
        k3[i].v = k2[i].v + (deltaT * k2[i].a * 0.5f);
    }
    for (size_t i = 0; i < particles.size(); i++)
    {
        Particle* p = particles[i];
        k3[i].F = getForce(p, k2[i].x, p->m_);
        k3[i].a = k3[i].F / p->m_;
    }

    // K4
    for (size_t i = 0; i < particles.size(); i++)
    {
        k4[i].x = k3[i].x + (deltaT * k3[i].v * 0.5f);
        k4[i].v = k3[i].v + (deltaT * k3[i].a * 0.5f);
    }
    for (size_t i = 0; i < particles.size(); i++)
    {
        Particle* p = particles[i];
        k4[i].F = getForce(p, k4[i].x, p->m_);
        k4[i].a = k4[i].F / p->m_;
    }

    // update values
    for (size_t i = 0; i < particles.size(); i++)
    {
        Particle* p = particles[i];
        sf::Vector2f x = p->x_ + (deltaT/6) * (k1[i].v + k2[i].v*2.f + k3[i].v*2.f + k4[i].v);
        sf::Vector2f v = p->v_ + (deltaT/6) * (k1[i].a + k2[i].a*2.f + k3[i].a*2.f + k4[i].a);

        p->setPosition(x);
        p->v_ = v;
    }
}

void ParticleDynamics::Euler(float deltaT)
{
    for (size_t i = 0; i < particles.size(); i++)
    {
        Particle* p = particles[i];
        sf::Vector2f F = getForce(p, p->x_, p->m_);
        sf::Vector2f a = F / p->m_;
        p->setPosition(p->x_ + deltaT * p->v_);
        p->v_ = p->v_ + deltaT * a; 
    }
}

void ParticleDynamics::update(float deltaT)
{
    // spaws particles at specified rate -> not part of ParticleDynamics implementation
    if (spawner_enabled_) SpriteSpawner::instance()->spawn(this);

    if (ParticleDynamics::rk4)
    {
        RK4(deltaT);
    }
    else
    {
        Euler(deltaT);
    }
}

void ParticleDynamics::draw(sf::RenderWindow& window, float delta_time)
{
    for (ForceSource* f : force_sources)
    {
        f->draw(window);
    }

    // update trail
    time_since_last_recording_ += delta_time;
    if (time_since_last_recording_ >= trail_seconds / 20.f)
    {  
        for (Particle* p : particles)
        {
            if (p->position_history_.size() == 20) p->position_history_.pop_front();
            p->position_history_.push_back(p->sprite_->getPosition());
            time_since_last_recording_ = 0.f;
        }
    }
    
    assert(level_lock.try_lock() == false);
    for (auto iter = this->particles.begin(); iter != this->particles.end();)
    {
        // delete particle if feature enabled and if too far away from screen center
        if ((*iter)->sprite_->checkDestroy(window))
        {
            auto force_iter = std::find(force_sources.begin(), force_sources.end(), *iter);
            if (force_iter != force_sources.end()) 
            {
                force_sources.erase(force_iter);
            }

            delete *iter;
            iter = this->particles.erase(iter);
            continue;
        }
        (*iter)->sprite_->draw(window);
        iter++;
    }
}

#define SAMPLES 15
void ParticleDynamics::drawTrail(sf::RenderWindow& window)
{
    if (!ParticleDynamics::draw_trails) return;

    for (Particle* p : particles)
    {
        sf::Vector2f previous = p->sprite_->getPosition();
        auto iter = p->position_history_.end()-2;
        for (; iter > p->position_history_.begin(); iter--)
        {
            sf::CircleShape circle;
            circle.setRadius(2);
            circle.setFillColor(sf::Color::Green);
            circle.setOrigin(2, 2);
            circle.setPosition(*iter);
            window.draw(circle);

            sf::Vertex line[] =
            {
                sf::Vertex(previous, sf::Color::Blue),
                sf::Vertex(*iter, sf::Color::Blue)
            };
            window.draw(line, 2, sf::Lines);
            previous = *iter;
        }
    }
}

#define UNIT 100.f
#define SAMPLES_PER_UNIT 4
#define DBS (UNIT/SAMPLES_PER_UNIT) // distance between sample

void ParticleDynamics::drawForceField(sf::RenderWindow& window, float object_mass)
{
    if (!ParticleDynamics::draw_ff) return;

    sf::Texture arrow_tex;
    arrow_tex.loadFromFile("res/arrow_red.png");

    int samples_x = (window.getSize().x / UNIT) * SAMPLES_PER_UNIT;
    int samples_y = (window.getSize().y / UNIT) * SAMPLES_PER_UNIT;

    // mass to calculate force field
    float m = object_mass;

    for (int x = 0; x < samples_x; x++)
    {
        for (int y = 0; y < samples_y; y++)
        {
            sf::Vector2f pos = {x*DBS + DBS/2, y*DBS + DBS/2};
            sf::Vector2f x = {(float) pos.x, (float) pos.y};
            sf::Vector2f F = getForce(nullptr, x, m);
            drawArrow(window, arrow_tex, pos, F);
        }
    }
}

#define MAX_MAGNITUDE 5000.f
void ParticleDynamics::drawArrow(sf::RenderWindow& window, sf::Texture& tex, sf::Vector2f position, sf::Vector2f F)
{
    sf::RectangleShape arrow;
    arrow.setTexture(&tex);
    arrow.setPosition(position);
    arrow.setSize({DBS, DBS});
    arrow.setOrigin({DBS/2, DBS/2});

    float magnitude = vlength(F);
    magnitude = magnitude > 0 ? magnitude : -magnitude;
    float orientation = atan2(F.y, F.x) * 180/M_PI;
    float scale = std::clamp(std::clamp(magnitude, 0.f, MAX_MAGNITUDE) / (MAX_MAGNITUDE), 0.3f, 1.f);

    arrow.rotate(orientation);
    arrow.setScale(sf::Vector2f(scale, scale));

    window.draw(arrow);
}