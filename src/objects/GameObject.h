#include "SFML/Graphics.hpp"

#ifndef MAGNITY_GO_H
#define MAGNITY_GO_H

class GameObject
{
public:
    sf::Vector2f position_;
    sf::Vector2f scale_;
    float rotation_;
    bool active_ = true;

    GameObject() {}

    GameObject(sf::Vector2f position, float rotation=0)
    {
        this->position_ = position;
        this->rotation_ = rotation;
        this->scale_ = {0, 0};
    }

    virtual ~GameObject() {}

    sf::Vector2f getPosition() { return position_; }
    virtual void setPosition(sf::Vector2f position) { this->position_ = position; }
    virtual void setScale(sf::Vector2f scale) { this->scale_ = scale_; }
    virtual void setRotation(float rotation) { this->rotation_ = rotation; }

    virtual void update(float delta_time) {};
    virtual void draw(sf::RenderWindow& window) {};
};

#endif // MAGNITY_GO_H