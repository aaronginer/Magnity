///////////////////////////////////////////////////////////////////////////////////////
//  1d. Rigid-Body Dynamics                                                         //
//  Mathematical Concepts were taken from the lecture slides, as well as            //
//  the script "Physically Based Modeling by David Baraff from TeachCenter          //
///////////////////////////////////////////////////////////////////////////////////////

#ifndef MAGNITY_CONTACT_H
#define MAGNITY_CONTACT_H
#include <SFML/Graphics.hpp>
#include "RigidBody.h"

class Contact {
public:
    Contact(RigidBody *a, RigidBody *b, sf::Vector3f n, sf::Vector3f p);

    RigidBody *a; //body containing vertex
    RigidBody *b; //body containing face
    sf::Vector3f n; //outwards pointing normal of face
    sf::Vector3f p; //world-space vertex location
    sf::Vector3f ea; //edge direction A
    sf::Vector3f eb; //edge direction for B
    bool vf; //true if vertex/face contact
    void computeContactForces(Contact contacts[], int ncontacts, double t);
    sf::Vector3f computeNdot(Contact *c); //return derivative of the normal vector
    double compute_aij(Contact ci, Contact cj);
    bool colliding(Contact& c); //return true if bodies are colliding - set THRESHOLD
    void collision(Contact& c, double epsilon); //loop through all contact points
    void FindAllCollisions(Contact& contact, int ncontacts);
    void compute_a_matrix(Contact contacts[], int ncontacts, std::vector<std::vector<double>> &a);
    void compute_b_vector(Contact contacts[], int ncontacts, std::vector<double> &b);
    sf::Vector3f pt_velocity(RigidBody& body, sf::Vector3f& p);
    void DetectAndResolveCollision(Contact& contact);
    sf::Vector3f CalculateImpulse(Contact& contact);
    void ApplyImpulse(RigidBody &body, const sf::Vector3f & impulse, Contact& contact);
    sf::Vector3f CalculateRelativeVelocity(Contact& contact);
};


#endif //MAGNITY_CONTACT_H
