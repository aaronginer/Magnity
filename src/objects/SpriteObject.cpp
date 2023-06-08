#include "SpriteObject.h"

SpriteObject::SpriteObject(sf::Texture& texture, sf::Vector2f position, float rotation) : GameObject(position, rotation) 
{
    this->sprite_.setTexture(texture); 
    this->sprite_.setPosition(this->position_);

    sf::FloatRect bounds = this->sprite_.getGlobalBounds();
    this->sprite_.setOrigin({bounds.width/2, bounds.height/2});
}

void SpriteObject::setScale(sf::Vector2f scale)
{
    this->scale_ = scale;
    this->sprite_.setScale(scale);
 
    sf::FloatRect bounds = this->sprite_.getGlobalBounds();
    this->sprite_.setOrigin({bounds.width/2, bounds.height/2});
}

void SpriteObject::setPosition(sf::Vector2f position)
{
    this->position_ = position;
    this->sprite_.setPosition(this->position_);
}

void SpriteObject::setRotation(float rotation)
{
    this->rotation_ = rotation;
    this->sprite_.setRotation(this->rotation_);
}

void SpriteObject::move(sf::Vector2f mov)
{
    this->sprite_.move(mov);
    this->position_ = this->sprite_.getPosition();
}

bool SpriteObject::contains(sf::Vector2f pos)
{
    sf::FloatRect bounds = this->sprite_.getGlobalBounds();
    return bounds.contains(pos);
}

void SpriteObject::draw(sf::RenderWindow& window)
{
    if (!this->active_) return;
    
    window.draw(this->sprite_);
}