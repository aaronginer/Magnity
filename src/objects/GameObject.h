#include "SFML/Graphics.hpp"

#ifndef MAGNITY_GO_H
#define MAGNITY_GO_H

class GameObject
{
public:
    sf::Vector2f position_;
    float rotation_;
    bool active_ = true;

    GameObject(sf::Vector2f position, float rotation=0)
    {
        this->position_ = position;
        this->rotation_ = rotation;
    }

    virtual ~GameObject() {}

    sf::Vector2f getPosition() { return position_; }

    virtual void draw(sf::RenderWindow& window) {};
};

#endif // MAGNITY_GO_H