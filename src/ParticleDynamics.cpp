#include "ParticleDynamics.h"
#include "cmath"
#include "algorithm"

#define M_PI 3.14159265358979323846
#define PIXELS_PER_UNIT 100000

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
    this->state.x = x;
    this->state.v = v;
    this->state.m = m;
    this->sprite_ = new SpriteObject(texture, x);
}

sf::Vector2f ForceSource::getForce(sf::Vector2f x /*pos*/, float m /*m*/)
{
    // gravitational softening length
    #define EPSILON (1.0f/100)

    float multiplier = 1.f;
    switch (type)
    {
        case ForceType::AntiGravity:
        {
            multiplier = -1.f;
        }
        case ForceType::Gravity:
        {
            sf::Vector2f r = this->x - x;
            float d = vlength(r) / PIXELS_PER_UNIT;
            if (d > 0.0001f) r /= d; // normalize

            // force magnitude at distance d
            float g = G * (this->m / (pow(d, 2) + pow(EPSILON, 2)));
            // g = 9.81f;

            return (r * g * m) * multiplier;
        }
        case ForceType::VectorField:
        {
            switch (vf_func)
            {
                /*case Sin:
                    return {20*s.m, (float) 100*sin(s.x.x/100)*s.m};
                case NegSin:
                    return {-20*s.m, (float) 100*sin(s.x.x/100)*s.m};*/
                case Defined:
                {
                    return {0, this->m * ((int)(x.x / 100) % 2 == 0 ? -1 : 1)}; // placeholder
                }   
                default:
                    return {this->m, this->m}; // placeholder
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
    if (type != ForceType::Gravity && type == ForceType::AntiGravity) return;

    sf::CircleShape c;
    c.setPosition(x);
    c.setFillColor(sf::Color(255*m/50000.f, 0, 0));
    c.setOrigin({10,10});
    c.setRadius(10);
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
        k1[i].x = p->state.x;
        k1[i].v = p->state.v;
    }
    // calculate forces and update acceleration
    for (size_t i = 0; i < particles.size(); i++)
    {
        Particle* p = particles[i];
        k1[i].F = getForce(p, k1[i].x, p->state.m);
        k1[i].a = k1[i].F / p->state.m;
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
        k2[i].F = getForce(p, k2[i].x, p->state.m);
        k2[i].a = k2[i].F / p->state.m;
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
        k3[i].F = getForce(p, k2[i].x, p->state.m);
        k3[i].a = k3[i].F / p->state.m;
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
        k4[i].F = getForce(p, k4[i].x, p->state.m);
        k4[i].a = k4[i].F / p->state.m;
    }

    // update values
    for (size_t i = 0; i < particles.size(); i++)
    {
        Particle* p = particles[i];
        sf::Vector2f x = p->state.x + (deltaT/6) * (k1[i].v + k2[i].v*2.f + k3[i].v*2.f + k4[i].v);
        sf::Vector2f v = p->state.v + (deltaT/6) * (k1[i].a + k2[i].a*2.f + k3[i].a*2.f + k4[i].a);

        p->setPosition(x);
        p->state.v = v;
    }
}

void ParticleDynamics::Euler(float deltaT)
{
    for (size_t i = 0; i < particles.size(); i++)
    {
        Particle* p = particles[i];
        sf::Vector2f F = getForce(p, p->state.x, p->state.m);
        sf::Vector2f a = F / p->state.m;
        p->setPosition(p->state.x + deltaT * p->state.v);
        p->state.v = p->state.v + deltaT * a; 
    }
}

void ParticleDynamics::update(float deltaT)
{
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
    

    for (Particle* p : particles)
    {
        p->sprite_->draw(window);
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
#define SAMPLES_PER_UNIT 3
#define DBS (UNIT/SAMPLES_PER_UNIT) // distance between sample

void ParticleDynamics::drawForceField(sf::RenderWindow& window, float object_mass)
{
    if (!ParticleDynamics::draw_ff) return;

    sf::Texture arrow_tex;
    arrow_tex.loadFromFile("res/arrow_red.png");

    int samples_x = (window.getSize().x / UNIT) * SAMPLES_PER_UNIT;
    int samples_y = (window.getSize().y / UNIT) * SAMPLES_PER_UNIT;

    ParticleState s;
    s.m = object_mass;

    for (int x = 0; x < samples_x; x++)
    {
        for (int y = 0; y < samples_y; y++)
        {
            sf::Vector2f pos = {x*DBS + DBS/2, y*DBS + DBS/2};
            s.x = {(float) pos.x, (float) pos.y};
            sf::Vector2f F = getForce(nullptr, s.x, s.m);
            drawArrow(window, arrow_tex, pos, F);
        }
    }
}

#define MAX_MAGNITUDE 500.f
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