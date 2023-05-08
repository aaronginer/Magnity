#include "Magnet.h"

Magnet::Magnet(sf::Texture *texture, int player) {
    this->magnet.setTexture(texture);
    this->magnet.setSize(Vector2f(40.0f, 50.0f));
    if(player == 1) {
        this->magnet.setPosition(0.0f, 0.0f);
    } else {
        this->magnet.setPosition(0.0f, 400.0f);
    }

    // this->magnet.setOrigin(this->magnet.getSize() / 2.0f);
    this->player = player;
}

Magnet::~Magnet() {

}

Vector2f Magnet::getPosition() {
    return this->magnet.getPosition();
}

void Magnet::Draw(sf::RenderWindow &window) {
    window.draw(this->magnet);
}

RectangleShape Magnet::getMagnet() {
    return this->magnet;
}
