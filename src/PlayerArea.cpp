//
// Created by Laura Pessl on 30.04.23.
//

#include <iostream>
#include "PlayerArea.h"

PlayerArea::PlayerArea(sf::Texture *texture, RenderWindow &window, int player) {
    area.setSize(Vector2f(window.getSize().x, 150.0f));
    if(player == 1) {
        area.setPosition(0.0f, 0.0f);
    }
    else {
        area.setPosition(0.0f, window.getSize().y - 380.0f);
    }
    area.setTexture(texture);
    // area.setOrigin(area.getSize() / 2.0f);

    //std::cout << "Postion: x " << area.getPosition().x << " y " << area.getPosition().y << std::endl;
}

PlayerArea::~PlayerArea() {

}

Vector2f PlayerArea::getPosition() {
    return area.getPosition();
}

void PlayerArea::Draw(RenderWindow &window) {
    area.setSize(Vector2f(window.getSize().x, 150.0f));
    window.draw(area);
}

RectangleShape PlayerArea::getArea() {
    return area;
}
