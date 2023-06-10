#include "SFML/Graphics.hpp"
#include "../RigidBody.h"

#pragma once

class WallArea {
public:
    std::vector<sf::RectangleShape> areas_;

    WallArea();
    ~WallArea();

    void load(std::string file_path);
    void draw(sf::RenderWindow& window);
};