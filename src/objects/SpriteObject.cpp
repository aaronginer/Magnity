#include "SpriteObject.h"
#include "cmath"

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
    
    if (flipping_enabled_)
    {
        current_time_ += clock_.restart();
        if (current_time_ > sampling_rate_)
        {
            bool move_left = prev_position_.x > this->position_.x;
            float scale_x = scale_.x;
            if (scale_x > 0 && move_left) scale_x = -scale_x;
            else if (scale_x < 0 && !move_left) scale_x = -scale_x;

            setScale({scale_x, scale_.y});
            prev_position_ = this->position_;
            current_time_ = sf::seconds(0);
        }
    }
        
    window.draw(this->sprite_);
}

bool SpriteObject::checkDestroy(sf::RenderWindow& window)
{
    if (!destroy_if_too_far_) return false;

    sf::Vector2f v = window.getView().getCenter() - this->getPosition(); 

    if (sqrt(pow(v.x, 2) + pow(v.y, 2)) > 1000) return true;
    return false;
}