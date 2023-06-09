#include "SpriteObject.h"

SpriteObject::SpriteObject(sf::Texture& texture, sf::Vector2f position, int origin) : GameObject(position, 0) 
{
    this->sprite_.setTexture(texture); 
    this->sprite_.setPosition(this->position_);

    this->setOrigin(origin);
}

void SpriteObject::setScale(sf::Vector2f scale)
{
    this->scale_ = scale;
    this->sprite_.setScale(scale);
 
    this->setOrigin(this->origin_);
}

void SpriteObject::setOrigin(int origin)
{   
    this->origin_ = origin;
    sf::FloatRect bounds = this->sprite_.getLocalBounds();
    switch (this->origin_)
    {
        case 0: // center
            this->sprite_.setOrigin({bounds.width/2, bounds.height/2});
            break;
        case 1: // top left
            this->sprite_.setOrigin({0, 0});
            break;
        case 2: // top right
            this->sprite_.setOrigin({bounds.width, 0});
            break;
        case 3: // bottom left
            this->sprite_.setOrigin({0, bounds.height});
            break;
        case 4: // bottom right
            this->sprite_.setOrigin({bounds.width, bounds.height});
            break;
        default:
            this->sprite_.setOrigin({bounds.width/2, bounds.height/2});
            break;
    }
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