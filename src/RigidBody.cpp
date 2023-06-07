#include "RigidBody.h"
#include <iostream>
#include <SFML/Graphics.hpp>
#include "math.h"

//array of rigid bodies
unsigned int highest_id = 0;

//TODO: see forum for Rigid Body

RigidBody::RigidBody(double mass, double density, unsigned int type, double width, double height, const sf::Texture& texture,
                     bool fixed, double posX, double posY) {
    this->type = type;
    this->fixed = fixed;
    this->height = height;
    this->width = width;
    this->mass = mass;
    if(type == 0) {
        radius = this->height / 2.0f;
        this->body.setOrigin((float)radius, (float)radius);
        this->Inertia = (this->mass * (radius * radius)) / 2.0;
        //FROM WIKIPEDIA
    }
    else {
        this->body.setOrigin((float)width / 2.0f, (float)height / 2.0f);
        this->Inertia = (this->mass * ((this->width * this->width) + (this->height + this->height))) / 12;

        radius = this->height / 2.0f;
        this->body.setOrigin((float)radius, (float)radius);
        this->Inertia = (this->mass * (radius * radius)) / 2.0;
        //FROM WIKIPEDIA
    }
    this->body.setSize({(float)height, (float)width});
    this->body.setTexture(&texture);
    this->x = sf::Vector3<double>(posX, posY, 0.0f);
    this->body.setPosition((float)posX, (float)posY);
    highest_id++;
    this->Ibody = this->calcIbody();
    this->P = sf::Vector3<double>(0,0,0);
    this->L = 0.0;
    this->angular_acceleration = 0.0;
    this->w = 0.0f;
    this->torque = 0.0f;
    this->collision_found = false;
}

void RigidBody::DisplayBodies(sf::RenderWindow &window, std::vector<RigidBody*> *rigid_bodies) {
    //loop through all bodies and display them
    for (auto & rigid_bodie : *rigid_bodies) {
        window.draw(rigid_bodie->body);
    }
}

double RigidBody::calcMagnitude(sf::Vector3<double> vec) {
    return std::sqrt((vec.x * vec.x) + (vec.y * vec.y));
}

sf::Vector3<double> RigidBody::calcCrossProd(sf::Vector3<double> vec1, sf::Vector3<double> vec2) {
    return {0.0f, 0.0f, (vec1.x * vec2.y) - (vec1.y * vec2.x)};
}


void RigidBody::ComputeForceAndTorque(RigidBody *rb) {
    //compute all forces
    //TODO: check where player magnates are and compute force accordingly
    if(rb->type == 1) {
        rb->force = sf::Vector3<double>(rb->force.x, -100.00f * rb->mass, rb->force.z);
    }
    else {
        rb->force = sf::Vector3<double>(rb->force.x, 150.0f * rb->mass, rb->force.z);
    }

    //TODO: here we need to check for collisions!

    std::cout << std::endl;
}

Matrix RigidBody::calcIbody() const {
    //Formulas taken from https://en.wikipedia.org/wiki/List_of_moments_of_inertia
    if(this->type == 1) //is rectangular block
    {
        //take inetia tensor formular for blocks - since its only 2D - z = 0
        return {(this->height * this->height) * this->mass * (1.0f/12.0f), 0.0f, 0.0f, 0.0f,
                (this->width * this->width) * this->mass * (1.0f/12.0f), 0.0f, 0.0f, 0.0f,
                ((this->width * this->width)+(this->height * this->height)) * this->mass * (1.0f / 12.0f)};
    }
    else {
        return {(this->radius * this->radius) * (this->mass) * (2.0f / 5.0f), 0.0f, 0.0f, 0.0f,
                (this->radius * this->radius) * (this->mass) * (2.0f / 5.0f), 0.0f, 0.0f, 0.0f,
                (this->radius * this->radius) * (this->mass) * (2.0f / 5.0f)};
    }
}

void RigidBody::applyVelocityVerletIntegration(RigidBody* rigid_body0, RigidBody* rigid_body1, double timestep) {
    //1. Update Position
    sf::Vector3<double> tmp_velocity = sf::Vector3<double>(rigid_body0->v.x * timestep, rigid_body0->v.y * timestep, rigid_body0->v.z * timestep);

    double timestep2 = (timestep * timestep);

    sf::Vector3<double> tmp = sf::Vector3<double>(0.5 * rigid_body0->linear_acceleration.x * timestep2, 0.5 * rigid_body0->linear_acceleration.y * timestep2,
                                    0.5 * rigid_body0->linear_acceleration.z * timestep2);

    //get next position of rigid body
    rigid_body1->x = rigid_body0->x + sf::Vector3<double>(tmp_velocity.x + tmp.x, tmp_velocity.y + tmp.y, tmp_velocity.z + tmp.z);

    //calculate forces, the NOW act on rigid body
    ComputeForceAndTorque(rigid_body1);

    //2. Update Acceleration
    rigid_body1->linear_acceleration = rigid_body1->force / rigid_body1->mass;

    //3. Update linear Velocity
    sf::Vector3<double> sum_accelerations = (rigid_body0->linear_acceleration + rigid_body1->linear_acceleration);
    rigid_body1->v = rigid_body0->v + sf::Vector3<double>(0.5 * sum_accelerations.x * timestep, 0.5 * sum_accelerations.y * timestep,
                                                          0.5 * sum_accelerations.z * timestep);
}

//x0 = init state vector
//len = len of x0
//t0 = starting time
//t1 = end time
//solver needs to compute the state vector at time t1 and return it in xEnd
//function dxdt -> has array x that encodes state vector x(t), it must returnd d/dt x(t) in the xdot
//we need t in dxdt because we may have time varying forces
//ode can call dxdt as often as it likes
void RigidBody::ode(std::vector<RigidBody*> *y0, std::vector<RigidBody> *yEnd, int len, double t0, double t1, std::vector<RigidBody*>* rigid_bodies, std::vector<Border*> obstacles) {
    //Compute current state of object

    double timestep = t1 - t0;

    for(int i = 0; i < y0->size(); i++) {
        //STEPS
        //1. Use Velocity Verlet Integration to get new Position/Acceleration
        // apply forces and update linear
        std::cout << std::endl;
        applyVelocityVerletIntegration(y0->at(i), &yEnd->at(i), timestep);
        //rigid bodies were updated -> now check for collisions at new position
        if(!yEnd->at(i).collision_found) {
            yEnd->at(i).checkForCollisions(rigid_bodies, obstacles);
        }
        else {
            yEnd->at(i).collision_found = false;
        }

        //Update y0

        //2. update angular momentum, angular acceleration, angular velocity
        yEnd->at(i).angular_acceleration = y0->at(i)->torque_vec.z / y0->at(i)->Inertia;
        yEnd->at(i).w += y0->at(i)->angular_acceleration * timestep;

        //TODO: is this damping good?
        float damping = 0.01f * yEnd->at(i).mass; // Adjust this value to control the rate of damping
        yEnd->at(i).w -= damping * yEnd->at(i).w * timestep;

        y0->at(i)->body.setPosition(yEnd->at(i).x.x, yEnd->at(i).x.y);
        y0->at(i)->body.rotate(yEnd->at(i).w);


        //save old state
        y0->at(i)->x = yEnd->at(i).x;
        y0->at(i)->torque_vec = yEnd->at(i).torque_vec;
        y0->at(i)->force_points = yEnd->at(i).force_points;
        y0->at(i)->Inertia = yEnd->at(i).Inertia;
        y0->at(i)->P = yEnd->at(i).P;
        y0->at(i)->angular_acceleration = yEnd->at(i).angular_acceleration;
        y0->at(i)->w = yEnd->at(i).w;
        y0->at(i)->force = yEnd->at(i).force;
        y0->at(i)->linear_acceleration = yEnd->at(i).linear_acceleration;
        y0->at(i)->q = yEnd->at(i).q;
        y0->at(i)->v = yEnd->at(i).v;
        y0->at(i)->L = yEnd->at(i).L;

    }

}

sf::Vector3<double> RigidBody::normalizeVector(sf::Vector3<double> vec) {
    if(calcMagnitude(vec) == 0) {
        return {0.0f, 0.0f, 0.0f};
    }

    return {(double)(vec.x / calcMagnitude(vec)), (double)(vec.y / calcMagnitude(vec)),(double)(vec.z / calcMagnitude(vec))};
}

void RigidBody::checkForCollisions(std::vector<RigidBody*> *rigid_bodies, std::vector<Border*> obstacles) {
    //go through all objects and check for collisions
    //TODO: make this easier with just the distance between circles
    //TODO: with border just check y coordinate
    for (int i = 0; i < rigid_bodies->size(); i++) {

        //check if object is touching borders / obstacles
        for (int j = 0; j < obstacles.size(); j++) {
            double distance = ::fabsf(rigid_bodies->at(i)->x.y - obstacles.at(j)->getPosition().y);
            if (distance <
                (rigid_bodies->at(i)->radius + (obstacles.at(j)->getShape().getSize().y))) { //TODO: look at this again

                if (obstacles.at(j)->getCollisionPointDir() == 0) { // this bottom border

                    rigid_bodies->at(i)->x.y =
                            obstacles.at(j)->getPosition().y + (obstacles.at(j)->getShape().getSize().y) +
                            rigid_bodies->at(i)->radius;

                    sf::Vector3<double> obstacle3d = sf::Vector3<double>(obstacles.at(j)->getPosition().x,
                                                                         obstacles.at(j)->getPosition().y, 0.0f);

                    //calculate normal
                    sf::Vector3<double> normal = normalizeVector(rigid_bodies->at(i)->x - obstacle3d);
                    sf::Vector3<double> normal2 = sf::Vector3<double>(normal.x * normal.x, normal.y * normal.y,
                                                                      normal.z * normal.z);
                    //calculate impulse -> new velocities
                    sf::Vector3<double> oldVelocity1 = rigid_bodies->at(i)->v - sf::Vector3<double>(0.0f, 0.0f, 0.0f);

                    double divMass1 = (2 * 500) / (rigid_bodies->at(i)->mass + 500);

                    sf::Vector3<double> v1_v2 = oldVelocity1; //velocity of v2 is 0 - it is fixed

                    sf::Vector3<double> right_side1 = sf::Vector3<double>(v1_v2.x * divMass1, v1_v2.y * divMass1,
                                                                          v1_v2.z * divMass1);
                    right_side1 = sf::Vector3<double>(right_side1.x * normal2.x, right_side1.y * normal2.y,
                                                      right_side1.z * normal2.z);

                    rigid_bodies->at(i)->v = oldVelocity1 - right_side1;
                } else if (obstacles.at(j)->getCollisionPointDir() == 2) { // top border

                    rigid_bodies->at(i)->x.y =
                            obstacles.at(j)->getPosition().y - (obstacles.at(j)->getShape().getSize().y) -
                            rigid_bodies->at(i)->radius;

                    sf::Vector3<double> obstacle3d = sf::Vector3<double>(obstacles.at(j)->getPosition().x,
                                                                         obstacles.at(j)->getPosition().y, 0.0f);

                    //calculate normal
                    sf::Vector3<double> normal = normalizeVector(rigid_bodies->at(i)->x - obstacle3d);
                    sf::Vector3<double> normal2 = sf::Vector3<double>(normal.x * normal.x, normal.y * normal.y,
                                                                      normal.z * normal.z);
                    //calculate impulse -> new velocities
                    sf::Vector3<double> oldVelocity1 = rigid_bodies->at(i)->v - sf::Vector3<double>(0.0f, 0.0f, 0.0f);

                    double divMass1 = (2 * 500) / (rigid_bodies->at(i)->mass + 500);

                    sf::Vector3<double> v1_v2 = oldVelocity1; //velocity of v2 is 0 - it is fixed

                    sf::Vector3<double> right_side1 = sf::Vector3<double>(v1_v2.x * divMass1, v1_v2.y * divMass1,
                                                                          v1_v2.z * divMass1);
                    right_side1 = sf::Vector3<double>(right_side1.x * normal2.x, right_side1.y * normal2.y,
                                                      right_side1.z * normal2.z);

                    rigid_bodies->at(i)->v = oldVelocity1 - right_side1;
                }
            }
        }


        //check if object is touching other rigid body
        for (int j = 0; j < rigid_bodies->size(); j++) {
            if (j == i) {
                continue;
            }

            double distance = std::sqrt(std::pow(rigid_bodies->at(j)->x.x - rigid_bodies->at(i)->x.x, 2) +
                                        std::pow(rigid_bodies->at(j)->x.y - rigid_bodies->at(i)->x.y, 2));


            if (distance <= (rigid_bodies->at(i)->radius + rigid_bodies->at(j)->radius)) {

                /*while (distance < (rigid_bodies->at(i)->radius + rigid_bodies->at(j)->radius)) {
                    rigid_bodies->at(i)->x.x = rigid_bodies->at(i)->x.x - (rigid_bodies->at(i)->v.x * 0.0001);
                    rigid_bodies->at(i)->x.y = rigid_bodies->at(i)->x.y - (rigid_bodies->at(i)->v.y * 0.0001);
                    distance = std::sqrt(std::pow(rigid_bodies->at(j)->x.x - rigid_bodies->at(i)->x.x, 2) +
                                         std::pow(rigid_bodies->at(j)->x.y - rigid_bodies->at(i)->x.y, 2));
                }*/

                while (distance < (rigid_bodies->at(i)->radius + rigid_bodies->at(j)->radius)) {
                    rigid_bodies->at(i)->x.x = rigid_bodies->at(i)->x.x - (rigid_bodies->at(i)->v.x * 0.0001f);
                    rigid_bodies->at(i)->x.y = rigid_bodies->at(i)->x.y - (rigid_bodies->at(i)->v.y * 0.0001f);
                    distance = std::sqrt(std::pow(rigid_bodies->at(j)->x.x - rigid_bodies->at(i)->x.x, 2) +
                                         std::pow(rigid_bodies->at(j)->x.y - rigid_bodies->at(i)->x.y, 2));
                }

                //sleep(5);


                /*sf::Vector3<double> collision_point = sf::Vector3<double>(
                        rigid_bodies->at(i)->x.x + ((distance - rigid_bodies->at(i)->radius) *
                                                    (rigid_bodies->at(j)->x.x - rigid_bodies->at(i)->x.x)
                                                    / (rigid_bodies->at(i)->radius + rigid_bodies->at(j)->radius)),
                        rigid_bodies->at(i)->x.y + ((distance - rigid_bodies->at(i)->radius) *
                                                    (rigid_bodies->at(j)->x.y - rigid_bodies->at(i)->x.y)
                                                    / (rigid_bodies->at(i)->radius + rigid_bodies->at(j)->radius)), 0.0f
                );*/

                sf::Vector3<double> collision_point = (rigid_bodies->at(i)->x * rigid_bodies->at(j)->radius +
                                                       rigid_bodies->at(j)->x * rigid_bodies->at(i)->radius) /
                                                      (rigid_bodies->at(i)->radius + rigid_bodies->at(j)->radius);

                //calculate normal
                sf::Vector3<double> normal = normalizeVector(rigid_bodies->at(i)->x - rigid_bodies->at(j)->x);

                sf::Vector3<double> normal2 = sf::Vector3<double>(normal.x * normal.x, normal.y * normal.y,
                                                                  normal.z * normal.z);
                //calculate impulse -> new velocities
                sf::Vector3<double> v1_v2 = rigid_bodies->at(i)->v - rigid_bodies->at(j)->v;

                double divMass1 =
                        (2 * rigid_bodies->at(j)->mass) / (rigid_bodies->at(i)->mass + rigid_bodies->at(j)->mass);
                double divMass2 =
                        (2 * rigid_bodies->at(i)->mass) / (rigid_bodies->at(i)->mass + rigid_bodies->at(j)->mass);


                sf::Vector3<double> right_side1 = sf::Vector3<double>(v1_v2.x * divMass1, v1_v2.y * divMass1,
                                                                      v1_v2.z * divMass1);
                right_side1 = sf::Vector3<double>(right_side1.x * normal2.x, right_side1.y * normal2.y,
                                                  right_side1.z * normal2.z);

                sf::Vector3<double> right_side2 = sf::Vector3<double>(v1_v2.x * divMass2, v1_v2.y * divMass2,
                                                                      v1_v2.z * divMass2);
                right_side2 = sf::Vector3<double>(right_side2.x * normal2.x, right_side2.y * normal2.y,
                                                  right_side2.z * normal2.z);

                //calculate J = j * n for Torque
                sf::Vector3<double> r1 = collision_point - rigid_bodies->at(i)->x;
                sf::Vector3<double> r2 = collision_point - rigid_bodies->at(j)->x;

                //-------------------------------------- from old code -----------------------------------------
                sf::Vector3<double> p1 = rigid_bodies->at(i)->v + calcCrossProd(r1, sf::Vector3<double>(0.0, 0.0, rigid_bodies->at(i)->w));
                sf::Vector3<double> p2 = rigid_bodies->at(j)->v + calcCrossProd(r1, sf::Vector3<double>(0.0, 0.0, rigid_bodies->at(j)->w));

                sf::Vector3<double> p1_p2 = p1 - p2;

                double dividend = -2 * ( (p1_p2.x * normal.x) + (p1_p2.y * normal.y) + (p1_p2.z * normal.z) );
                double divisor = std::pow(rigid_bodies->at(i)->mass, -1) + std::pow(rigid_bodies->at(j)->mass, -1);

                double j_imp = dividend / divisor;

                sf::Vector3<double> J = sf::Vector3<double>(normal.x * j_imp, normal.y * j_imp, normal.z * j_imp);
                //----------------------------------------------------------------------------------------------

                //update linear momentum
                //rigid_bodies->at(i)->P += right_side1;
                //rigid_bodies->at(j)->P -= right_side2;

                std::cout << "Before Linear velocity  Rigid body " << i << "  (" << rigid_bodies->at(i)->v.x << ", " << rigid_bodies->at(i)->v.y << ", " << rigid_bodies->at(i)->v.z << ")" << std::endl;
                std::cout << "Before Linear velocity  Rigid body " << j << "  (" << rigid_bodies->at(j)->v.x << ", " << rigid_bodies->at(j)->v.y << ", " << rigid_bodies->at(j)->v.z << ")" << std::endl;

                rigid_bodies->at(i)->v = rigid_bodies->at(i)->v + J;
                rigid_bodies->at(j)->v = rigid_bodies->at(j)->v  - J;

                //set torque vector
                rigid_bodies->at(i)->torque_vec = calcCrossProd(r1, right_side1);
                rigid_bodies->at(j)->torque_vec = calcCrossProd(r2, -right_side2);

                //update angular momentum
                rigid_bodies->at(i)->L += rigid_bodies->at(i)->torque_vec.z;
                rigid_bodies->at(j)->L += rigid_bodies->at(j)->torque_vec.z;

                rigid_bodies->at(j)->collision_found = true;

                std::cout << "R1 " << "  (" << r1.x << ", " << r1.y << ", " << r1.z << ")" << std::endl;
                std::cout << "R2 " << "  (" << r2.x << ", " << r2.y << ", " << r2.z << ")" << std::endl;
                std::cout << "right_side1 " << "  (" << right_side1.x << ", " << right_side1.y << ", " << right_side1.z << ")" << std::endl;
                std::cout << "right_side2 " << "  (" << right_side2.x << ", " << right_side2.y << ", " << right_side2.z << ")" << std::endl;
                std::cout << "Linear velocity  Rigid body " << i << "  (" << rigid_bodies->at(i)->v.x << ", " << rigid_bodies->at(i)->v.y << ", " << rigid_bodies->at(i)->v.z << ")" << std::endl;
                std::cout << "Linear velocity  Rigid body " << j << "  (" << rigid_bodies->at(j)->v.x << ", " << rigid_bodies->at(j)->v.y << ", " << rigid_bodies->at(j)->v.z << ")" << std::endl;
                std::cout << "Torque  Rigid body " << i << "  (" << rigid_bodies->at(i)->torque_vec.x << ", " << rigid_bodies->at(i)->torque_vec.y << ", " << rigid_bodies->at(i)->torque_vec.z << ")" << std::endl;
                std::cout << "Torque  Rigid body " << j << "  (" << rigid_bodies->at(j)->torque_vec.x << ", " << rigid_bodies->at(j)->torque_vec.y << ", " << rigid_bodies->at(j)->torque_vec.z << ")" << std::endl;
                std::cout << "Angular momentum  Rigid body " << i << "  = " << rigid_bodies->at(i)->L << std::endl;
                std::cout << "Angular momentum  Rigid body " << j << "  = " << rigid_bodies->at(j)->L << std::endl;

                //sleep(5);

                std::cout << "-----------------------------" << std::endl;
            }
        }
    }
}
