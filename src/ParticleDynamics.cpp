#include "ParticleDynamics.h"
#include "cmath"
#include "algorithm"

#define M_PI 3.14159265358979323846
#define PIXELS_PER_UNIT 10000

// static float distance(sf::Vector2f v1, sf::Vector2f v2)
// {
//     return sqrt(pow(v1.x - v2.x, 2) + pow(v1.y - v2.y, 2));
// }

static float vlength(sf::Vector2f v)
{
    return sqrt(pow(v.x, 2) + pow(v.y, 2));
}

Particle::Particle(sf::Texture& texture, sf::Vector2f x, sf::Vector2f v, float m)
{
    this->state.x = x;
    this->state.v = v;
    this->state.m = m;
    this->sprite_ = new SpriteObject(texture, x);
}

sf::Vector2f ForceSource::getForce(ParticleState& s)
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
            sf::Vector2f r = x - s.x;
            float d = vlength(r) / PIXELS_PER_UNIT;
            r /= d; // normalize

            // force magnitude at distance d
            float g = G * (m / (pow(d, 2) + pow(EPSILON, 2)));
            // g = 9.81f;

            return (r * g * s.m) * multiplier;
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
                    return {0, m * ((int)(s.x.x / 100) % 2 == 0 ? -1 : 1)};
                }   
                default:
                    return {m, m};
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
    c.setFillColor(sf::Color::Red);
    c.setOrigin({10,10});
    c.setRadius(10);
    window.draw(c);
}

bool ParticleDynamics::draw_trails = false;
bool ParticleDynamics::draw_ff = false;
int ParticleDynamics::trail_seconds = 2;

void ParticleDynamics::updateForce(ParticleState& s)
{
    sf::Vector2f F = {0, 0};
    for (ForceSource* f : force_sources)
    {
        F += f->getForce(s);
    }

    s.F = F;
}

void ParticleDynamics::RK4(ParticleState& s, float deltaT)
{
    ParticleState state1 = s, state2 = s, state3 = s, state4 = s;
    sf::Vector2f a1, a2, a3, a4;

    updateForce(state1);
    a1 = state1.F / s.m;
    
    state2.x = state1.x + (state1.v * deltaT * 0.5f);
    state2.v = state1.v + (a1 * deltaT * 0.5f);
    updateForce(state2);
    a2 = state2.F / s.m;

    state3.x = state1.x + (state2.v * deltaT * 0.5f);
    state3.v = state1.v + (a2 * deltaT * 0.5f);
    updateForce(state3);
    a3 = state3.F / s.m;
    
    state4.x = state1.x + (state3.v * deltaT);
    state4.v = state1.v + (a3 * deltaT);
    updateForce(state4);
    a4 = state4.F / s.m;

    s.x = s.x + (deltaT/6) * (state1.v + state2.v*2.f + state3.v*2.f + state4.v);
    s.v = s.v + (deltaT/6) * (a1 + a2*2.f + a3*2.f + a4);
    updateForce(s);
}

void ParticleDynamics::Euler(ParticleState& s, float deltaT)
{
    updateForce(s);
    sf::Vector2f a = s.F / s.m;
    s.x = s.x + s.v * deltaT;
    s.v = s.v + a * deltaT;
}

void ParticleDynamics::particleUpdate(ParticleState& s, float deltaT)
{
    if (rk4)
    {
        ParticleDynamics::RK4(s, deltaT);
    }
    else
    {
        ParticleDynamics::Euler(s, deltaT);
    }
}

void ParticleDynamics::update(float deltaT)
{
    for (Particle* p : particles)
    {
        particleUpdate(p->state, deltaT);
        p->sprite_->setPosition(p->state.x);
    }
}

void ParticleDynamics::draw(sf::RenderWindow& window, float delta_time)
{
    for (ForceSource* f : force_sources)
    {
        f->draw(window);
    }

    for (Particle* p : particles)
    {
        time_since_last_recording_ += delta_time;
        if (time_since_last_recording_ >= trail_seconds / 20.f)
        {
            if (p->position_history_.size() == 20) p->position_history_.pop_front();
            p->position_history_.push_back(p->sprite_->getPosition());
            time_since_last_recording_ = 0.f;
        }

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
            updateForce(s);
            drawArrow(window, arrow_tex, pos, s.F);
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