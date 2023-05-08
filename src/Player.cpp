//
// Created by Laura Pessl on 30.04.23.
//

#include "Player.h"

Player::Player(sf::Texture *texture, sf::Vector2u imgCount, float switchTime, float speed) :
        animation(texture, imgCount, switchTime)
{
    this->speed = speed;
    this->row = 0;
    this->faceRight = true;

    body.setSize(Vector2f(185.0f, 170.0f));
    body.setPosition(206.0f, 206.0f);
    body.setTexture(texture);
    body.setOrigin(body.getSize() / 2.0f);
}

Player::~Player() {

}

void Player::Update(float deltaTime) {
    Vector2f movement(0.0f, 0.0f);

    if(Keyboard::isKeyPressed(Keyboard::A)) {
        movement.x -= speed * deltaTime;
    }

    if(Keyboard::isKeyPressed(Keyboard::D)) {
        movement.x += speed * deltaTime;
    }

    if(movement.x == 0.0f) {
        row = 0;
    } else {
        row = 1;

        if(movement.x > 0.0f) {
            faceRight = true;
        } else {
            faceRight = false;
        }
    }

    animation.Update(row, deltaTime, faceRight);
    body.setTextureRect(animation.uvRect);
    body.move(movement);
}

void Player::Draw(sf::RenderWindow &window) {
    window.draw(body);
}

Vector2f Player::getPosition() {
    return body.getPosition();
}
