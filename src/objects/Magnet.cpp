#include "Magnet.h"
#include "cmath"

#define M_PI 3.14159265358979323846

Magnet::Magnet(sf::Texture& texture, sf::Vector2f position, int player) : SpriteObject(texture, position) {
    
    setScale({0.1f, 0.1f});
    this->player = player;
}

Magnet::~Magnet() {

}

void Magnet::setFollowObject(GameObject* follow_object)
{
    this->follow_object_ = follow_object;
}

void Magnet::updateRotation()
{
    if (this->follow_object_ == nullptr) return;

    sf::Vector2f v = follow_object_->getPosition() - getPosition();
    float rotation = atan2(v.y, v.x) * 180/M_PI;

    setRotation(rotation);
}


void Magnet::move(sf::Vector2f mov)
{
    SpriteObject::move(mov);
}
