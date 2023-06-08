#include "GameObject.h"

#ifndef MAGNITY_SPRITEO_H
#define MAGNITY_SPRITEO_H

class SpriteObject : public GameObject {
public:
    sf::Sprite sprite_;

    SpriteObject(sf::Texture& texture, sf::Vector2f position, float rotation=0);

    virtual ~SpriteObject() {}

    void setScale(sf::Vector2f scale) override;
    void setRotation(float rotation) override;
    void setPosition(sf::Vector2f position) override;
    void move(sf::Vector2f mov);
    
    bool contains(sf::Vector2f pos);

    sf::Sprite& getSprite() { return this->sprite_; }
    void draw(sf::RenderWindow& window) override;
};

#endif // MAGNITY_SPRITEO_H