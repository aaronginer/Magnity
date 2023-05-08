//
// Created by Laura on 07.05.2023.
//

#ifndef MAGNITY_CONTACT_H
#define MAGNITY_CONTACT_H
#include <SFML/Graphics.hpp>


class Contact {
public:
    Contact();

    //RigidBody *a;
    //RigidBody *b; //body containing face
    sf::Vector3f n; //outwards pointing normal of face
    sf::Vector3f p; //world-space vertex location
    sf::Vector3f ea; //edge direction A
    sf::Vector3f eb; //edge direction for B
    bool vf; //true if vertex/face contact

};


#endif //MAGNITY_CONTACT_H
