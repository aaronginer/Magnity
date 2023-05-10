#include <iostream>
#include "Contact.h"

Contact::Contact(RigidBody *a, RigidBody *b, sf::Vector3f n, sf::Vector3f p) {
    this->a = a;
    this->b = b;
    this->n = n;
    this->p = p;
}

/*bool Contact::colliding(Contact& c) {
     sf::Vector3f padot = pt_velocity(*c.a, c.p);
     sf::Vector3f pbdot = pt_velocity(*c.b, c.p);
     c.n = {0.0f, 1.0f, 0.0f};
     double vrel = c.a->calcDotProd(c.n, (padot - pbdot)); //TODO: can i write - like that?
     //std::cout << "verl Beneath: " << vrel << std::endl;
     if(vrel > 1.0f) //moving away
     {
         return true;
     }
     else if(vrel > -1.0f) //resting contact
     {
         return true;
     }
     else {

        return true;
     }
}

void Contact::collision(Contact& c, double epsilon) {
     sf::Vector3f padot = pt_velocity(*c.a, c.p);
     sf::Vector3f pbdot = pt_velocity(*c.b, c.p);
     c.n = {0.0f, 1.0f, 0.0f};
     sf::Vector3f n = c.n;
     sf::Vector3f ra = p - c.a->x;
     sf::Vector3f rb = p - c.b->x;
     double vrel = RigidBody::calcDotProd(n, padot - pbdot);

     vrel = -2.0f;
     double numerator = -(1 + epsilon) * vrel;

     //Calculate denominator in four parts
     double term1 = 1.0f / c.a->mass;
     double term2 = 1.0f / c.b->mass;
     double term3 = RigidBody::calcDotProd(n,
                                      (RigidBody::calcCrossProd(RigidBody::calcMatrixMult(c.a->Iinv, RigidBody::calcCrossProd(ra, n)), ra)));
     double term4 = RigidBody::calcDotProd(n,
                                     (RigidBody::calcCrossProd(RigidBody::calcMatrixMult(c.b->Iinv, RigidBody::calcCrossProd(rb, n)), rb)));

     //Compute the impulse magnitude
     double j = numerator / (term1 + term2 + term3 + term4);
     sf::Vector3f force = sf::Vector3f((float)(n.x * j), (float)(n.y * j), (float)(n.z * j));

     //Apply the impulse to the bodies
     c.a->P += force;
     c.b->P -= force;
     c.a->L += RigidBody::calcCrossProd(ra, force);
     c.b->L -= RigidBody::calcCrossProd(rb, force);

     //recompute auxiliary variables
     c.a->v = sf::Vector3f((float)(c.a->P.x / c.a->mass), (float)(c.a->P.y / c.a->mass), (float)(c.a->P.z / c.a->mass));
     c.b->v = sf::Vector3f((float)(c.b->P.x / c.b->mass), (float)(c.b->P.y / c.b->mass), (float)(c.b->P.z / c.b->mass));
     c.a->omega = RigidBody::calcMatrixMult(c.a->Iinv, c.a->L);
     c.b->omega = RigidBody::calcMatrixMult(c.b->Iinv, c.b->L);
}

void Contact::FindAllCollisions(Contact& contact, int ncontacts) {

     bool had_collision;
     double epsilon = 0.5;

    // do {
         had_collision = false;

         for(int i = 0; i < ncontacts; i++) {
             if(colliding(contact)) {
                 collision(contact, epsilon);
                 had_collision = true;

                 //Tell the solver we had a collision
                 //ode_discontinues(); //TODO: what is that?
             }
         }
     //} while(had_collision);
}

void Contact::computeContactForces(Contact *contacts, int ncontacts, double t) {
     //we assume that every element of contacts[] represents a contact in resting contact
     //Also we assume that for each element in rigid_bodies[] the force and torque fileds
     //have been set to the net external force and torque acting on the body due to gravity,
     //wind etc. perhaps by a call to//computeExternalForceAndTorqueForAllBodies(t); //TODO: implement this method?

     //allocate nxn matrix amat and n-vectors fvec and bvec

    std::vector<std::vector<double>> *amat;
     //Matrix *amat = new Matrix(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
     std::vector<double> *bvec = new std::vector<double>(ncontacts); //TODO: is this really filled with doubles?
     std::vector<double> *fvec = new std::vector<double>(ncontacts); //TODO: is this really filled with doubles?

     //compute aij and bi coefficients
     compute_a_matrix(contacts, ncontacts, *amat);
     compute_b_vector(contacts, ncontacts, *bvec);

     //solve for fj's
     //qp_solve(*amat, *bvec, *fvec);

     //Now add the resting contact forces we jsut computed into the force and torque field of each body
     for(int i = 0; i < ncontacts; i++) {
         double f = fvec->at(i); //fi
         sf::Vector3f n = contacts[i].n;
         RigidBody *A = contacts[i].a;
         RigidBody *B = contacts[i].b;

         //apply the force f n positively to A
         A->force += sf::Vector3f(n.x * f, n.y * f, n.z * f);
         A->torque += sf::Vector3f((contacts[i].p - A->x).x * sf::Vector3f(n.x * f, n.y * f, n.z * f).x,
                                   (contacts[i].p - A->x).y * sf::Vector3f(n.x * f, n.y * f, n.z * f).y,
                                   (contacts[i].p - A->x).z * sf::Vector3f(n.x * f, n.y * f, n.z * f).z);

         //and negatively to B
         B->force -= sf::Vector3f(n.x * f, n.y * f, n.z * f);
         B->torque -=  sf::Vector3f((contacts[i].p - B->x).x * sf::Vector3f(n.x * f, n.y * f, n.z * f).x,
                                    (contacts[i].p - B->x).y * sf::Vector3f(n.x * f, n.y * f, n.z * f).y,
                                    (contacts[i].p - B->x).z * sf::Vector3f(n.x * f, n.y * f, n.z * f).z);
     }
}

sf::Vector3f Contact::computeNdot(Contact *c) {
     if(c->vf) //vertex/face contact
     {
         //the vector n is attached to B so..
         return c->a->calcCrossProd(c->b->omega, c->n);
     }
     else {
         sf::Vector3f eadot = c->a->calcCrossProd(c->a->omega, c->ea);
         sf::Vector3f ebdot = c->a->calcCrossProd(c->b->omega, c->eb);
         sf::Vector3f n1 =  c->a->calcCrossProd(c->ea, c->eb); //TODO is this really a cross product?
         sf::Vector3f z = c->a->calcCrossProd(eadot, eb) + c->a->calcCrossProd(ea, ebdot);
         double l = c->a->calcMagnitude(n1);
         n1 = c->a->calcDivScalar(n1, l);

         return (z - (c->a->calcDivScalar(c->a->calcCrossProd(c->a->calcCrossProd(z, c->n), c->n),  l)));
     }
}

void Contact::compute_b_vector(Contact *contacts, int ncontacts, std::vector<double> &b) {
     for(int i = 0; i < ncontacts; i++) {
         Contact *c = &contacts[i];
         RigidBody *A = c->a;
         RigidBody *B = c->b;
         sf::Vector3f n = c->n;
         sf::Vector3f ra = c->p - A->x;
         sf::Vector3f rb = c->p - B->x;

         //Get the external forces and torques
         sf::Vector3f f_ext_a = A->force;
         sf::Vector3f f_ext_b = B->force;
         sf::Vector3f t_ext_a = A->torque;
         sf::Vector3f t_ext_b = B->torque;

         sf::Vector3f a_ext_part, a_vel_part, b_ext_part, b_vel_part;

         //compute the part of p..a(t0) due to the external force and torque
         a_ext_part = sf::Vector3f(f_ext_a.x / A->mass, f_ext_a.y / A->mass, f_ext_a.z / A->mass)
                 + c->a->calcCrossProd(c->a->calcMatrixMult(A->Iinv, t_ext_a), ra);
         b_ext_part = sf::Vector3f(f_ext_b.x / B->mass, f_ext_b.y / B->mass, f_ext_b.z / B->mass)
                      + c->a->calcCrossProd(c->a->calcMatrixMult(B->Iinv, t_ext_b), rb);

         //Compute the part of p..a(to) due to velocity
         a_vel_part = c->a->calcCrossProd(A->omega, (c->a->calcCrossProd(A->omega, ra))) +
                 c->a->calcCrossProd((c->a->calcMatrixMult(A->Iinv, (c->a->calcCrossProd(A->L, A->omega)))), ra);
         b_vel_part = c->b->calcCrossProd(B->omega, (c->b->calcCrossProd(B->omega, rb))) +
                      c->b->calcCrossProd((c->b->calcMatrixMult(B->Iinv, (c->b->calcCrossProd(B->L, B->omega)))), rb);

         //combine the above results and dot with nî(t0)
         double k1 = c->a->calcDotProd(((a_ext_part + a_vel_part) - (b_ext_part + b_vel_part)), n);
         sf::Vector3f ndot = computeNdot(c);
         double k2 = c->a->calcDotProd(sf::Vector3f(ndot.x * 2, ndot.y * 2, ndot.z * 2),
                                 ((pt_velocity(*A, c->p) - pt_velocity(*B, c->p))));

         b[i] = k1 + k2;

     }
}

void Contact::compute_a_matrix(Contact *contacts, int ncontacts, std::vector<std::vector<double>> &a) {
     for(int i = 0; i < ncontacts; i++) {
         for(int j = 0; j < ncontacts; j++) {
             a.at(i).at(j) = compute_aij(contacts[i], contacts[j]);
         }
     }
}

double Contact::compute_aij(Contact ci, Contact cj) {
     //if the bodies involved in the ith and jth contact are distinct, then aij is zero
     if((ci.a != cj.a) && (ci.b != cj.b) && (ci.a != cj.b) && (ci.b != cj.a)) {
         return 0.0;
     }

     RigidBody *A = ci.a;
     RigidBody *B = ci.b;
     sf::Vector3f ni = ci.n;
     sf::Vector3f nj = cj.n;
     sf::Vector3f pi = ci.p;
     sf::Vector3f pj = cj.p;
     sf::Vector3f ra = pi - A->x;
     sf::Vector3f rb = pi - B->x;

     sf::Vector3f force_on_a = sf::Vector3f(0.0f, 0.0f, 0.0f);
     sf::Vector3f torque_on_a = sf::Vector3f(0.0f, 0.0f, 0.0f);

     if(cj.a == ci.a) {
         //force direction of jth contact force on A
         force_on_a = nj;
         //torque direction
         torque_on_a = ci.a->calcCrossProd(pj - A->x, nj);
     } else if(cj.b == ci.a) {
         force_on_a = sf::Vector3f(nj.x * -1, nj.y * -1, nj.z * -1);
         torque_on_a = ci.a->calcCrossProd(pj - A->x, nj);
     }

     sf::Vector3f force_on_b = sf::Vector3f(0.0f, 0.0f, 0.0f);
     sf::Vector3f torque_on_b = sf::Vector3f(0.0f, 0.0f, 0.0f);

     if(cj.a == ci.b) {
         //force direction of jth contact force on A
         force_on_b = nj;
         //torque direction
         torque_on_b = ci.b->calcCrossProd(pj - B->x, nj);
     } else if(cj.b == ci.b) {
         force_on_b = sf::Vector3f(nj.x * -1, nj.y * -1, nj.z * -1);
         torque_on_b = ci.b->calcCrossProd(pj - B->x, nj);
     }

     //Now compute how the jth contact force affects the linear and angular acceleration of the
     //contact point on body A
     sf::Vector3f a_linear = sf::Vector3f(force_on_a.x / A->mass, force_on_a.y / A->mass, force_on_a.z / A->mass);
     sf::Vector3f a_angular = ci.a->calcCrossProd(ci.a->calcMatrixMult(A->Iinv, torque_on_a), ra);
     //Same for B
     sf::Vector3f b_linear = sf::Vector3f(force_on_b.x / B->mass, force_on_b.y / B->mass, force_on_b.z / B->mass);
     sf::Vector3f b_angular = ci.b->calcCrossProd(ci.b->calcMatrixMult(B->Iinv, torque_on_b), rb);

     return ci.a->calcDotProd(ni, ((a_linear + a_angular) - (b_linear + b_angular)));
}

//TODO: can i make + like that?
sf::Vector3f Contact::pt_velocity(RigidBody& body, sf::Vector3f& p) {
    if(body.fixed){
        return sf::Vector3f(0.0f, 0.0f, 0.0f);
    }
    else {
        return body.v + body.calcCrossProd(body.omega, sf::Vector3f(p.x - body.world_coord.x, p.y - body.world_coord.y, p.z - body.world_coord.z));
    }
}
*/
//-------------------------------Different approach -------------------------------------------------------------------

// Detect a collision between two rigid bodies and apply the necessary impulses
void Contact::DetectAndResolveCollision(Contact& contact) {

    //sf::Vector3f tmp = contact.a->calcCrossProd((contact.p - contact.a->world_coord), (contact.p - contact.b->world_coord));
    //tmp.z = 1;
    // Calculate the contact point and normal
    sf::Vector3f normal = contact.a->normalizeVector(contact.b->world_coord - contact.a->world_coord);
    contact.n = normal;

    // Calculate and apply the impulses to resolve the collision
    sf::Vector3f impulse = contact.CalculateImpulse(contact);

    // exit(0);
    contact.ApplyImpulse(*contact.a, impulse, contact);
    contact.ApplyImpulse(*contact.b, (impulse * -1.0f), contact);
}

sf::Vector3f Contact::CalculateImpulse(Contact &contact) { //THIS

    float e = 0.8f; // coefficient of restitution
    double j = (-(1.0f + e) * contact.a->calcDotProd(CalculateRelativeVelocity(contact),contact.n))
               /
               ((1.0f / contact.a->mass) + (1.0f / contact.b->mass));

    return {(float)(contact.n.x * j), (float)(contact.n.y * j), float(contact.n.z * j)};
}

void Contact::ApplyImpulse(RigidBody &body, const sf::Vector3f & impulse, Contact& contact) {
    body.P.x += impulse.x;
    body.P.y += impulse.y;
    body.P.z += impulse.z;
    body.L += body.calcCrossProd((contact.p - body.world_coord), impulse);
    body.v = body.calcDivScalar(body.P, body.mass);
    body.omega = body.calcMatrixMult(body.Iinv, body.L);
}

sf::Vector3f Contact::CalculateRelativeVelocity(Contact& contact) {
    sf::Vector3f r1 = contact.p;
    sf::Vector3f v1 = contact.a->P + contact.a->calcCrossProd(contact.a->L, contact.p - contact.a->world_coord);
    sf::Vector3f v2 = contact.b->P + contact.b->calcCrossProd(contact.b->L, contact.p - contact.b->world_coord);

    return v2 - v1;
}

