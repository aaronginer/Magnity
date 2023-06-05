#include "GameObject.h"

#ifndef MAGNITY_SPRITEO_H
#define MAGNITY_SPRITEO_H

class SpriteObject : public GameObject {
public:
    sf::Sprite sprite_;

    SpriteObject(sf::Texture& texture, sf::Vector2f position, float rotation=0) : GameObject(position, rotation) 
    {
        this->sprite_.setTexture(texture); 
        this->sprite_.setPosition(this->position_);
    }

    void setScale(sf::Vector2f scale);
    void setRotation(float rotation);
    void setPosition(sf::Vector2f scale);

    virtual void move(sf::Vector2f mov);
    
    sf::Sprite& getSprite() { return this->sprite_; }
    void draw(sf::RenderWindow& window) override;
};

#endif // MAGNITY_SPRITEO_H