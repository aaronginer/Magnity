#include "SpriteSpawner.h"
#include "../ParticleDynamics.h"

SpriteSpawner* SpriteSpawner::instance_ = nullptr;

#define WIDTH 1200
#define HEIGTH 800

SpriteSpawner::SpriteSpawner()
{
    spawn_range_x = {0, WIDTH};
    spawn_range_y = {0, 0};
}

void SpriteSpawner::loadNewTexture(std::string file_path)
{
    SpriteSpawner::instance()->texture.loadFromFile(file_path);
}

void SpriteSpawner::setSpawnRange(sf::Vector2i x, sf::Vector2i y)
{
    SpriteSpawner::instance()->spawn_range_x = x;
    SpriteSpawner::instance()->spawn_range_y = y;
}

void SpriteSpawner::setDrag(float k)
{
    SpriteSpawner::instance()->k_ = k;
}

void SpriteSpawner::setSpawnSpeed(float s)
{
    SpriteSpawner::instance()->time_between_spawns = sf::seconds(s);
}

SpriteSpawner* SpriteSpawner::instance()
{
    if (SpriteSpawner::instance_ == nullptr)
    {
        SpriteSpawner::instance_ = new SpriteSpawner();
    }

    return SpriteSpawner::instance_;
}

void  SpriteSpawner::spawn(ParticleDynamics* pdyn)
{
    if (pdyn == nullptr) return;
    if (SpriteSpawner::instance()->disabled) return;

    SpriteSpawner* spawner = SpriteSpawner::instance();


    sf::Time delta = spawner->clock.restart();
    spawner->current_time_spawns += delta;
   
    if (spawner->current_time_spawns >= spawner->time_between_spawns)
    {
        float spawn_x = spawner->spawn_range_x.x + std::rand() % abs(spawner->spawn_range_x.x - spawner->spawn_range_x.y);
        float spawn_y = spawner->spawn_range_y.x + std::rand() % abs(spawner->spawn_range_y.x - spawner->spawn_range_y.y);

        Particle* p = new Particle(spawner->texture, {spawn_x, spawn_y}, {0, 0}, 1);
        p->sprite_->setScale({0.7, 0.7});
        p->sprite_->setRotation(std::rand() % 360);
        p->sprite_->destroy_if_too_far_ = true;
        p->K_ = spawner->k_;
        pdyn->addParticle(p);
        pdyn->addForceSource(p);
        spawner->current_time_spawns = sf::seconds(0);
    }
}

void SpriteSpawner::disable()
{
    SpriteSpawner::instance()->disabled = true;
} 
void SpriteSpawner::enable()
{
    SpriteSpawner::instance()->disabled = false;
} 