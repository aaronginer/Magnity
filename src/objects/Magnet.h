#include <SFML/Graphics.hpp>
#include "SpriteObject.h"
#include "../ParticleDynamics.h"
using namespace sf;

#ifndef MAGNITY_MAGNET_H
#define MAGNITY_MAGNET_H


class Magnet : public SpriteObject {
public:
    Particle* follow_object_ = nullptr;
    
    Magnet(Texture& texture, sf::Vector2f position, int player);
    ~Magnet();

    void setFollowObject(Particle* follow_object);
    void updateRotation();
    void move(sf::Vector2f mov) override;
    int player;
};


#endif //MAGNITY_MAGNET_H
