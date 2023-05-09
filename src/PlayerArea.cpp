//
// Created by Laura Pessl on 30.04.23.
//

#include <iostream>
#include "PlayerArea.h"

PlayerArea::PlayerArea(sf::Texture *texture, View &view, int player) {
    area.setSize(Vector2f(view.getSize().x, 150.0f));
    if(player == 1) {
        area.setPosition(0.0f, 0.0f);
    }
    else {
        area.setPosition(0.0f, view.getSize().y - 150.0f);
    }
    area.setTexture(texture);
}

PlayerArea::~PlayerArea() {

}

Vector2f PlayerArea::getPosition() {
    return area.getPosition();
}

void PlayerArea::Draw(RenderWindow &window) {
    window.draw(area);
}

RectangleShape PlayerArea::getArea() {
    return area;
}
