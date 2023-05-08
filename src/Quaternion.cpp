//
// Created by Laura on 07.05.2023.
//

#include "Quaternion.h"

Quaternion::Quaternion() {
    this->r = 0.0f;
    this->i = 0.0f;
    this->j = 0.0f;
    this->k = 0.0f;
}

Quaternion::Quaternion(double r, double i, double j, double k) {
    this->r = r;
    this->i = i;
    this->j = j;
    this->k = k;
}

Quaternion::Quaternion(double r, sf::Vector3f vec) {
    this->r = r;
    this->i = vec.x;
    this->j = vec.y;
    this->k = vec.z;
}
