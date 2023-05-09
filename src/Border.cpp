//
// Created by Laura Pessl on 30.04.23.
//

#include "Border.h"

Border::Border(Texture *texture, const RectangleShape& shape, RenderWindow &window, int pos, bool fullscreen) {

    switch(pos) {
        case 0: //left
            if(fullscreen) {
                border.setSize(Vector2f(window.getSize().y, 10.0f));
            }
            else {
                border.setSize(Vector2f(shape.getSize().y, 10.0f));
            }
            border.rotate(90);
            border.setPosition(shape.getPosition().x, shape.getPosition().y);
            this->xORy = false;
            break;
        case 1: //top
            if(fullscreen) {
                border.setSize(Vector2f(window.getSize().x, 10.0f));
            }
            else {
                border.setSize(Vector2f(shape.getSize().x, 10.0f));
            }
            border.setPosition(shape.getPosition().x, shape.getPosition().y - (shape.getSize().y / 2.0f));
            this->xORy = true;
            break;
        case 2: //right
            if(fullscreen) {
                border.setSize(Vector2f(window.getSize().y, 10.0f));
            }
            else {
                border.setSize(Vector2f(shape.getSize().y, 10.0f));
            }
            border.setPosition(shape.getPosition().x, shape.getPosition().y);
            border.rotate(90);
            this->xORy = false;
            break;
        case 3: //bottom
            if(fullscreen) {
                border.setSize(Vector2f(window.getSize().x, 10.0f));
            }
            else {
                border.setSize(Vector2f(shape.getSize().x, 10.0f));
            }
            border.setPosition(shape.getPosition().x, shape.getPosition().y + (shape.getSize().y / 2.0f));
            border.rotate(180);
            this->xORy = true;
            break;
    }

    border.setTexture(texture);
    // border.setOrigin(border.getSize() / 2.0f);
    this->fullscreen = fullscreen;
}

Border::Border(sf::Texture *texture, int x, int y, float height, float width) {
    border.setSize(Vector2f(height, width));
    border.setPosition(x, y);
    border.setTexture(texture);
    this->fullscreen = false;
    this->xORy = false;
}

Border::~Border() {

}

Vector2f Border::getPosition() {
    return border.getPosition();
}

void Border::Draw(RenderWindow &window) {
    window.draw(border);
}