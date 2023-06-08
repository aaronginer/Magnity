#include <SFML/Graphics.hpp>
#include "SpriteObject.h"
#include "../ParticleDynamics.h"
#include "../gameplay/MagnetArea.h"
using namespace sf;

#ifndef MAGNITY_MAGNET_H
#define MAGNITY_MAGNET_H

struct MagnetKeySet {
    sf::Keyboard::Key move_left_;
    sf::Keyboard::Key move_right_;
    sf::Keyboard::Key move_up_;
    sf::Keyboard::Key move_down_;
    sf::Keyboard::Key toggle_level_;
};


class Magnet {
public:
    GameObject* follow_object_ = nullptr;
    int player_;
    int level_ = 1;
    MagnetKeySet key_set_;

    Magnet(MagnetKeySet key_set, sf::Vector2f position, int player);
    ~Magnet();

    std::vector<sf::Texture*> textures_;

    SpriteObject* magnet_inactive_ = nullptr;
    SpriteObject* magnet_active_ = nullptr;
    SpriteObject* levels_[3];
    ForceSource* force_source_ = nullptr;

    void handlePolledKeyInput(sf::Event keyEvent);
    void handleInstantKeyInput(float delta_time, MagnetArea* ma);

    void toggleLevel();
    void setFollowObject(GameObject* follow_object);
    void setForceSource(ForceSource* force_source_);
    void updateRotation();
    void move(sf::Vector2f mov);
    void draw(sf::RenderWindow& window);
};


#endif //MAGNITY_MAGNET_H
