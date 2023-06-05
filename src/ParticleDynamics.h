#include "SFML/Graphics.hpp"
#include "unistd.h"
#include <queue>

#ifndef MAGNITY_PARTICLES_H
#define MAGNITY_PARTICLES_H

#define G 0.0000000000667

class ParticleState {
public:
    // position
    sf::Vector2f x;
    // velocity
    sf::Vector2f v;
    // force
    sf::Vector2f F;
    // mass
    float m;
};

class Particle {
public:
    Particle(sf::Texture& texture, sf::Vector2f x, sf::Vector2f v, float m);

    sf::Sprite sprite;
    ParticleState state;
};

enum ForceType {
    Gravity,
    VectorField,
    AntiGravity,
    Constant
};

enum VectorFieldFunction {
    Sin,
    NegSin,
    Defined
};

class ForceSource {
public:
    ForceType type;
    // position
    sf::Vector2f x;
    // mass
    float m;
    // constant force
    sf::Vector2f F_ = {0, 0};

    VectorFieldFunction vf_func;

    ForceSource(ForceType type, sf::Vector2f x, float m)
    {
        this->type = type;
        this->x = x;
        this->m = m;
    }

    ForceSource(ForceType type, sf::Vector2f F)
    {
        this->type = type;
        this->F_ = F;
    }

    ForceSource(ForceType type, VectorFieldFunction vf_func)
    {
        this->type = type;
        this->vf_func = vf_func;
    }

    ForceSource(ForceType type, VectorFieldFunction vf_func, float strength)
    {
        this->type = type;
        this->vf_func = vf_func;
        this->m = strength;
    }

    ForceSource(ForceType type)
    {
        this->type = type;
    }


    sf::Vector2f getForce(ParticleState& s);
    void draw(sf::RenderWindow& window);
};

class ParticleDynamics {
public:
    static bool draw_trails;
    static bool draw_ff;
    
    bool rk4 = false;

    ParticleDynamics(bool rk4) { this->rk4 = rk4; }

    std::vector<Particle*> particles;
    std::vector<ForceSource*> force_sources;
    void updateForce(ParticleState& s);

    void RK4(ParticleState& s, float deltaT);
    void Euler(ParticleState& s, float deltaT);

    void addParticle(Particle* p) { particles.push_back(p); }
    void addForceSource(ForceSource* f) { force_sources.push_back(f); }

    void update(float deltaT);
    void particleUpdate(ParticleState& p, float deltaT);

    void draw(sf::RenderWindow& window);

    void drawTrail(sf::RenderWindow& window);
    void drawForceField(sf::RenderWindow& window, float object_mass=10.f);

private:
    void drawArrow(sf::RenderWindow& window, sf::Texture& tex, sf::Vector2f position, sf::Vector2f F);
};

#endif