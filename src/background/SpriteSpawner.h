#include "../objects/SpriteObject.h"
#include "../ParticleDynamics.h"
#include "SFML/Graphics.hpp"
#include "cstdlib"
#include "vector"


#pragma once

class SpriteSpawner {
private:
    sf::Time current_time_spawns = sf::seconds(0);
    sf::Clock clock;
    sf::Texture texture;
    SpriteSpawner();
public:    
    sf::Time time_between_spawns = sf::seconds(0.1f);
    bool disabled = true;
    sf::Vector2i spawn_range_x;
    sf::Vector2i spawn_range_y;
    float k_;
    static SpriteSpawner* instance_;
    static SpriteSpawner* instance();
    static void loadNewTexture(std::string file_path);
    static void setSpawnRange(sf::Vector2i x, sf::Vector2i y);
    static void setSpawnSpeed(float s);
    static void setDrag(float k);
    static void spawn(ParticleDynamics* pdyn); 
    static void disable(); 
    static void enable(); 
};