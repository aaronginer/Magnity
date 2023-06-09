#include "Magnet.h"

Magnet::Magnet(sf::Texture *texture, View& view, int player) {
    this->magnet.setTexture(texture);
    this->magnet.setSize(Vector2f(40.0f, 40.0f));
    if(player == 1) {
        this->magnet.setPosition(40.0f, 40.0f);
    } else {
        this->magnet.setPosition(40.0f, view.getSize().y - 40.f);
    }

    this->magnet.setOrigin(sf::Vector2f(20.f, 20.f));
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

RectangleShape& Magnet::getMagnet() {
    return this->magnet;
}
