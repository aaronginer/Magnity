//
// Created by Laura on 07.05.2023.
//

#include "Matrix.h"

Matrix::Matrix() {
    this->aa = 0.0f;
    this->ab = 0.0f;
    this->ac = 0.0f;
    this->ba = 0.0f;
    this->bb = 0.0f;
    this->bc = 0.0f;
    this->ca = 0.0f;
    this->cb = 0.0f;
    this->cc = 0.0f;
}

Matrix::Matrix(double aa, double ab, double ac, double ba, double bb, double bc, double ca, double cb, double cc) {
    this->aa = aa;
    this->ab = ab;
    this->ac = ac;
    this->ba = ba;
    this->bb = bb;
    this->bc = bc;
    this->ca = ca;
    this->cb = cb;
    this->cc = cc;
}