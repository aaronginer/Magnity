#include "SFML/Graphics.hpp"
#include "unistd.h"
#include <queue>

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

class ForceSource {
public:
    ForceType type;
    // position
    sf::Vector2f x;
    // mass
    float m;
    // constant force
    sf::Vector2f F_ = {0, 0};

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

    ForceSource(ForceType type)
    {
        this->type = type;
    }


    sf::Vector2f getForce(ParticleState& s);
    void draw(sf::RenderWindow& window);
};

class ParticleDynamics {
public:
    bool rk4 = false;

    ParticleDynamics(bool rk4) { this->rk4 = rk4; }

    std::vector<Particle> particles;
    std::vector<ForceSource*> force_sources;
    void updateForce(ParticleState& s);

    void RK4(ParticleState& s, float deltaT);
    void Euler(ParticleState& s, float deltaT);

    void addParticle(Particle p) { particles.push_back(p); }
    void addForceSource(ForceSource* f) { force_sources.push_back(f); }

    void update(float deltaT);
    void particleUpdate(ParticleState& p, float deltaT);

    void draw(sf::RenderWindow& window);

    void drawTrail(sf::RenderWindow& window);
    void drawForceField(sf::RenderWindow& window);
    void drawArrow(sf::RenderWindow& window, sf::Texture& tex, sf::Vector2f position, sf::Vector2f F);
};