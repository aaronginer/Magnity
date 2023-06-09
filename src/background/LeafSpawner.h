#include "../objects/SpriteObject.h"
#include "../ParticleDynamics.h"
#include "SFML/Graphics.hpp"
#include "cstdlib"
#include "vector"


#pragma once

class LeafSpawner {
private:
    sf::Time time_between_spawns = sf::seconds(0.4f);
    sf::Time current_time_spawns = sf::seconds(0);
    sf::Clock clock;
    sf::Texture texture;
    LeafSpawner();
public:    
    bool disabled = true;
    static LeafSpawner* instance_;
    static LeafSpawner* instance();
    static void spawnLeaf(ParticleDynamics* pdyn); 
    static void disable(); 
    static void enable(); 
};