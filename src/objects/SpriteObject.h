#include "GameObject.h"

#ifndef MAGNITY_SPRITEO_H
#define MAGNITY_SPRITEO_H

class SpriteObject : public GameObject {
public:
    sf::Sprite sprite_;
    sf::Vector2f prev_position_ = {0, 0};
    int origin_;
    bool destroy_if_too_far_ = false;

    sf::Time sampling_rate_ = sf::seconds(0.2f);
    sf::Time current_time_ = sf::seconds(0.0f);
    sf::Clock clock_;

    bool flipping_enabled_ = false;

    SpriteObject(sf::Texture& texture, sf::Vector2f position, int origin=0);

    virtual ~SpriteObject() {}

    sf::Sprite& getSprite() { return this->sprite_; }

    void setOrigin(int origin);
    void setScale(sf::Vector2f scale) override;
    void setRotation(float rotation) override;
    void setPosition(sf::Vector2f position) override;
    void move(sf::Vector2f mov);
    
    bool contains(sf::Vector2f pos);

    virtual void update(float delta_time) {};

    void draw(sf::RenderWindow& window) override;
    bool checkDestroy(sf::RenderWindow& window);
};

#endif // MAGNITY_SPRITEO_H