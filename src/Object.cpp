#include "Object.h"

Object::Object(sf::Texture *texture)
{
        body.setSize(Vector2f(14.0f, 20.0f));
        body.setPosition(206.0f, 206.0f);
        body.setTexture(texture);
        body.setOrigin(body.getSize() / 2.0f);
}

Object::~Object() {
}

Vector2f Object::getPosition() {
    return body.getPosition();
}

void Object::Draw(sf::RenderWindow &window) {
    window.draw(body);
}

void Object::Update(float deltaTime) {
}