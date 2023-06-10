#include "Magnet.h"
#include "cmath"
#define M_PI 3.14159265358979323846

Magnet::Magnet(MagnetKeySet key_set, sf::Vector2f position, int player)
{
    this->key_set_ = key_set;

    sf::Texture* active = new Texture();
    active->loadFromFile("res/magnet.png");
    sf::Texture* inactive = new Texture();
    inactive->loadFromFile("res/magnet_inactive.png");
    sf::Texture* level1 = new Texture();
    level1->loadFromFile("res/magnet_layer1.png");
    sf::Texture* level2 = new Texture();
    level2->loadFromFile("res/magnet_layer2.png");
    sf::Texture* level3 = new Texture();
    level3->loadFromFile("res/magnet_layer3.png");

    textures_.push_back(active);
    textures_.push_back(inactive);
    textures_.push_back(level1);
    textures_.push_back(level2);
    textures_.push_back(level3);
    
    this->magnet_active_ = new SpriteObject(*active, position);
    this->magnet_inactive_ = new SpriteObject(*inactive, position);
    this->levels_[0] = new SpriteObject(*level1, position);
    this->levels_[1] = new SpriteObject(*level2, position);
    this->levels_[2] = new SpriteObject(*level3, position);
    this->levels_[1]->active_ = false;
    this->levels_[2]->active_ = false;
    this->magnet_inactive_->active_ = false;

    this->magnet_active_->setScale({0.1f, 0.1f});
    this->magnet_inactive_->setScale({0.1f, 0.1f});
    this->levels_[0]->setScale({0.1f, 0.1f});
    this->levels_[1]->setScale({0.1f, 0.1f});
    this->levels_[2]->setScale({0.1f, 0.1f});

    this->player_ = player;
}

Magnet::~Magnet() {
    for (sf::Texture* t : this->textures_)
    {
        delete t;
    }

    delete this->magnet_active_;
    delete this->magnet_inactive_;

    delete this->levels_[0];
    delete this->levels_[1];
    delete this->levels_[2];
}

void Magnet::handlePolledKeyInput(sf::Event keyEvent)
{
    if (keyEvent.key.code == this->key_set_.toggle_level_)
    {
        toggleLevel();
    }
}

void Magnet::handleInstantKeyInput(float delta_time, MagnetArea* ma)
{
    if(Keyboard::isKeyPressed(this->key_set_.move_left_)) {
        sf::Vector2f move_vector = sf::Vector2f(-300.f*delta_time, 0.0);
        sf::Vector2f new_pos = this->magnet_active_->getPosition() + move_vector;
        if (ma->testLocationInBounds(new_pos)) move(move_vector);
    }
    if(Keyboard::isKeyPressed(this->key_set_.move_right_)) {
        sf::Vector2f move_vector = sf::Vector2f(300.f*delta_time, 0.0);
        sf::Vector2f new_pos = this->magnet_active_->getPosition() + move_vector;
        if (ma->testLocationInBounds(new_pos)) move(move_vector);
    }
    if(Keyboard::isKeyPressed(this->key_set_.move_down_)) {
        sf::Vector2f move_vector = sf::Vector2f(0.0, 300.f*delta_time);
        sf::Vector2f new_pos = this->magnet_active_->getPosition() + move_vector;
        if (ma->testLocationInBounds(new_pos)) move(move_vector);
    }
    if(Keyboard::isKeyPressed(this->key_set_.move_up_)) {
        sf::Vector2f move_vector = sf::Vector2f(0.0, -300.f*delta_time);
        sf::Vector2f new_pos = this->magnet_active_->getPosition() + move_vector;
        if (ma->testLocationInBounds(new_pos)) move(move_vector);
    }
}

void Magnet::toggleLevel()
{   
    this->level_ = (this->level_ + 1) % 4;
    
    this->magnet_active_->active_ = this->level_ != 0;    
    this->magnet_inactive_->active_ = this->level_ == 0;
    this->levels_[0]->active_ = this->level_ > 0;
    this->levels_[1]->active_ = this->level_ > 1;
    this->levels_[2]->active_ = this->level_ > 2;

    if (force_source_!= nullptr) this->force_source_->m_ = this->level_ * 300000;
}

void Magnet::setFollowObject(GameObject* follow_object)
{
    this->follow_object_ = follow_object;
}

void  Magnet::setForceSource(ForceSource* force_source_)
{
    this->force_source_ = force_source_;
    this->force_source_->x_ = this->magnet_active_->getPosition();
    this->force_source_->m_ = this->level_ * 300000;
}

void Magnet::updateRotation()
{
    if (this->follow_object_ == nullptr) return;

    sf::Vector2f v = follow_object_->getPosition() - magnet_active_->getPosition();
    float rotation = atan2(v.y, v.x) * 180/M_PI;

    this->magnet_active_->setRotation(rotation);
    this->magnet_inactive_->setRotation(rotation);
    this->levels_[0]->setRotation(rotation);
    this->levels_[1]->setRotation(rotation);
    this->levels_[2]->setRotation(rotation);
}


void Magnet::move(sf::Vector2f mov)
{
    this->magnet_active_->move(mov);
    this->magnet_inactive_->move(mov);
    this->levels_[0]->move(mov);
    this->levels_[1]->move(mov);
    this->levels_[2]->move(mov);

    if (force_source_ != nullptr) force_source_->x_ = this->magnet_active_->getPosition();
}

void Magnet::draw(sf::RenderWindow& window)
{
    this->magnet_active_->draw(window);
    this->magnet_inactive_->draw(window);
    this->levels_[0]->draw(window);
    this->levels_[1]->draw(window);
    this->levels_[2]->draw(window);
}