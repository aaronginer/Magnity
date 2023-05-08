//
// Created by Laura on 07.05.2023.
//

#ifndef MAGNITY_QUATERNION_H
#define MAGNITY_QUATERNION_H
#include <SFML/Graphics.hpp>


class Quaternion {
public:
    Quaternion();
    Quaternion(double r, double i, double j, double k);
    Quaternion(double r, sf::Vector3f vec);
    double r; //real part
    double i;
    double j;
    double k;

};


#endif //MAGNITY_QUATERNION_H
