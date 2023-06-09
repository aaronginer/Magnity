#include "SFML/Graphics.hpp"

#pragma once

class MagnetArea {
public:
    std::vector<sf::RectangleShape> areas_;
    std::vector<sf::RectangleShape> draw_areas_;
    std::vector<sf::Vector2f> spawn_points_;
    int magnets_ = 0;

    MagnetArea();
    ~MagnetArea();

    void load(std::string file_path);
    void draw(sf::RenderWindow& window);

    bool testLocationInBounds(sf::Vector2f location);
    sf::Vector2f getSpawnPoint();
};