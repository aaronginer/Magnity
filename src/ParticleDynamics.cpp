#include "ParticleDynamics.h"
#include "cmath"

static float distance(sf::Vector2f v1, sf::Vector2f v2)
{
    return sqrt(pow(v1.x - v2.x, 2) + pow(v1.y - v2.y, 2));
}

static float vlength(sf::Vector2f v)
{
    return sqrt(pow(v.x, 2) + pow(v.y, 2));
}

Particle::Particle(sf::Texture& texture, sf::Vector2f x, sf::Vector2f v, float m)
{
    this->state.x = x;
    this->state.v = v;
    this->state.m = m;
    this->sprite.setTexture(texture);
    this->sprite.setPosition(x);
}

sf::Vector2f ForceSource::getForce(ParticleState& s)
{
    sf::Vector2f r = x - s.x;
    float d = vlength(r);
    r /= d; // normalize

    float F_magnitude = G * (m * s.m) / pow(d, 2);
    // F_magnitude = 9.81f;

    //printf("%f (%f|%f) (%f|%f)\n", F_magnitude, (r*F_magnitude).x, (r*F_magnitude).y, r.x, r.y);
    return r * F_magnitude;//  * s.m;
}

sf::Vector2f ParticleDynamics::sumForces(ParticleState& s)
{
    sf::Vector2f F = {0, 0};
    for (ForceSource& f : force_sources)
    {
        F += f.getForce(s);
    }

    return F;
}

void ParticleDynamics::KT4(ParticleState& s, float deltaT)
{
    ParticleState state = s;
    sf::Vector2f a1, a2, a3, a4;
    sf::Vector2f v1, v2, v3, v4;
    sf::Vector2f F1, F2, F3, F4;

    F1 = sumForces(state);
    a1 = deltaT * (F1 / s.m);
    v1 = deltaT * s.v;

    state.x = s.x + v1 / 2.f;

    F2 = sumForces(state);
    a2 = deltaT * (F2 / s.m);
    v2 = deltaT * (s.v + a1 / 2.f);

    state.x = s.x + v2 / 2.f;

    F3 = sumForces(state);
    a3 = deltaT * (F3 / s.m);
    v3 = deltaT * (s.v + a2 / 2.f);

    state.x = s.x + v3;
}

void ParticleDynamics::Euler(ParticleState& s, float deltaT)
{
    sf::Vector2f F = sumForces(s);
    sf::Vector2f a = F / s.m;
    s.x = s.x + s.v * deltaT;
    s.v = s.v + a * deltaT;
}

void ParticleDynamics::particleUpdate(ParticleState& s, float deltaT)
{
    if (kt4)
    {
        ParticleDynamics::KT4(s, deltaT);
    }
    else
    {
        ParticleDynamics::Euler(s, deltaT);
    }
}

void ParticleDynamics::update(float deltaT)
{
    for (Particle& p : particles)
    {
        particleUpdate(p.state, deltaT);
        p.sprite.setPosition(p.state.x);
    }
}

void ParticleDynamics::draw(sf::RenderWindow& window)
{
    for (ForceSource& f : force_sources)
    {
        sf::CircleShape c;
        c.setPosition(f.x);
        c.setFillColor(sf::Color::Red);
        c.setRadius(10);
        window.draw(c);
    }

    for (Particle& p : particles)
    {
        window.draw(p.sprite);
    }
}

void ParticleDynamics::drawTrail(sf::RenderWindow& window)
{
    for (Particle& p : particles)
    {
        ParticleState s = p.state;
        for (int i = 0; i < 6; i++)
        {
            particleUpdate(s, -0.2f*i);

            sf::CircleShape circle;
            circle.setRadius(2);
            circle.setFillColor(sf::Color::Blue);
            circle.setPosition(s.x);
            window.draw(circle);
        }
    }
}