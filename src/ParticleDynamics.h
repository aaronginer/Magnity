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
    float F;
    // mass
    float m;
};

class Particle {
public:
    Particle(sf::Texture& texture, sf::Vector2f x, sf::Vector2f v, float m);

    sf::Sprite sprite;
    ParticleState state;
};

class ForceSource {
public:
    ForceSource(sf::Vector2f x, float m) { this->x = x; this->m = m; }
    // position
    sf::Vector2f x;
    // mass
    float m;

    sf::Vector2f getForce(ParticleState& s);
};

class ParticleDynamics {
public:
    bool kt4 = false;

    std::vector<Particle> particles;
    std::vector<ForceSource> force_sources;
    sf::Vector2f sumForces(ParticleState& s);

    void KT4(ParticleState& s, float deltaT);
    void Euler(ParticleState& s, float deltaT);

    void addParticle(Particle p) { particles.push_back(p); }
    void addForceSource(ForceSource f) { force_sources.push_back(f); }

    void update(float deltaT);
    void particleUpdate(ParticleState& p, float deltaT);

    void draw(sf::RenderWindow& window);

    void drawTrail(sf::RenderWindow& window);
};