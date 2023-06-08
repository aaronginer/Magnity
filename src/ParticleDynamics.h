#include "SFML/Graphics.hpp"
#include "unistd.h"
#include "objects/SpriteObject.h"
#include <deque>

#ifndef MAGNITY_PARTICLES_H
#define MAGNITY_PARTICLES_H

#define G 0.0000000000667

enum VectorFieldFunction {
    Sin,
    NegSin,
    Defined
};

enum ForceType {
    Gravity,
    VectorField,
    AntiGravity,
    Constant
};

class ForceSource {
public:
    ForceType type_;
    // position
    sf::Vector2f x_;
    // mass
    float m_;
    // constant force
    sf::Vector2f F_ = {0, 0};

    VectorFieldFunction vf_func;

    ForceSource(ForceType type, sf::Vector2f x, float m)
    {
        this->type_ = type;
        this->x_ = x;
        this->m_ = m;
    }

    ForceSource(ForceType type, sf::Vector2f F)
    {
        this->type_ = type;
        this->F_ = F;
    }

    ForceSource(ForceType type, VectorFieldFunction vf_func)
    {
        this->type_ = type;
        this->vf_func = vf_func;
    }

    ForceSource(ForceType type, VectorFieldFunction vf_func, float strength)
    {
        this->type_ = type;
        this->vf_func = vf_func;
        this->m_ = strength;
    }

    ForceSource(ForceType type)
    {
        this->type_ = type;
    }

    virtual sf::Vector2f getPosition() { return x_; }
    virtual void setPosition(sf::Vector2f new_pos) { x_ = new_pos; }
    sf::Vector2f getForce(sf::Vector2f x, float m);
    void draw(sf::RenderWindow& window);
};

class ParticleState {
public:
    
};

class Particle : public ForceSource {
public:
    SpriteObject* sprite_;
    std::deque<sf::Vector2f> position_history_;

    // position, force and mass are part fields of ForceSource

    // velocity
    sf::Vector2f v_;
    // drag
    float K_ = 0.47;

    Particle(sf::Texture& texture, sf::Vector2f x, sf::Vector2f v, float m);
    ~Particle() { delete sprite_; }

    sf::Vector2f getPosition() override { return x_; }
    void setPosition(sf::Vector2f pos) override { 
        x_ = pos;
        sprite_->setPosition(pos);
    }
};

class ParticleDynamics {
public:
    static bool draw_trails;
    static bool draw_ff;
    static int trail_seconds;
    static bool rk4;

    float time_since_last_recording_ = 0.0f;

    ParticleDynamics(bool rk4) { this->rk4 = rk4; }

    std::vector<Particle*> particles;
    std::vector<ForceSource*> force_sources;
    sf::Vector2f getForce(Particle* self, sf::Vector2f x, float m);

    void RK4(float deltaT);
    void Euler(float deltaT);

    void addParticle(Particle* p) { particles.push_back(p); }
    void addForceSource(ForceSource* f) { force_sources.push_back(f); }

    void update(float deltaT);
    void particleUpdate(ParticleState& p, float deltaT);

    void draw(sf::RenderWindow& window, float delta_time);

    void drawTrail(sf::RenderWindow& window);
    void drawForceField(sf::RenderWindow& window, float object_mass=10.f);

private:
    void drawArrow(sf::RenderWindow& window, sf::Texture& tex, sf::Vector2f position, sf::Vector2f F);
};

#endif // MAGNITY_PARTICLES_H