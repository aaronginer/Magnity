#include "LeafSpawner.h"
#include "../ParticleDynamics.h"

LeafSpawner* LeafSpawner::instance_ = nullptr;

#define WIDTH 1200
#define HEIGTH 800

LeafSpawner::LeafSpawner()
{
    texture.loadFromFile("res/leaf.png");
}

LeafSpawner* LeafSpawner::instance()
{
    if (LeafSpawner::instance_ == nullptr)
    {
        LeafSpawner::instance_ = new LeafSpawner();
    }

    return LeafSpawner::instance_;
}

void  LeafSpawner::spawnLeaf(ParticleDynamics* pdyn)
{
    if (pdyn == nullptr) return;
    if (LeafSpawner::instance()->disabled) return;

    LeafSpawner* spawner = LeafSpawner::instance();


    sf::Time delta = spawner->clock.restart();
    spawner->current_time_spawns += delta;
   
    if (spawner->current_time_spawns >= spawner->time_between_spawns)
    {
        Particle* p = new Particle(spawner->texture, {-100, -100 + std::rand() % (HEIGTH)}, {0, 0}, 1);
        p->sprite_->setScale({0.7, 0.7});
        p->sprite_->destroy_if_too_far_ = true;
        p->K_ = 20;
        pdyn->addParticle(p);
        pdyn->addForceSource(p);
        spawner->current_time_spawns = sf::seconds(0);
    }
}

void LeafSpawner::disable()
{
    LeafSpawner::instance()->disabled = true;
} 
void LeafSpawner::enable()
{
    LeafSpawner::instance()->disabled = false;
} 