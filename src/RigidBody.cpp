#include "RigidBody.h"
#include <iostream>
#include <SFML/Graphics.hpp>
#include "math.h"
#include "VoronoiFracture.h"
#include "objects/SpriteObject.h"

//array of rigid bodies
std::vector<RigidBody*> *RigidBody::rigid_bodies = new std::vector<RigidBody*>;
bool RigidBody::draw_vectors_ = false;
std::vector<RigidBody> *rigid_bodies_new = new std::vector<RigidBody>;
//TODO: see forum for Rigid Body

extern bool lost;
extern bool game_paused;
extern bool in_game;

RigidBody::RigidBody(double mass, double density, unsigned int type, double width, double height, const sf::Texture& texture,
                     bool fixed, double posX, double posY, int id) {
    this->type = type;
    this->id = id;
    this->height = height;
    this->width = width;
    this->mass = mass;
    if(type == 0) {
        radius = this->height / 2.0f;
        this->body.setOrigin((float)radius, (float)radius);
        this->Inertia = (this->mass * (radius * radius)) / 2.0;
        //FROM WIKIPEDIA
    }
    else if(this->type == 2 || this->type == 3) {
        this->body.setOrigin((float)width / 2.0f, (float )height / 2.0f);
        this->Inertia = (this->mass * ((this->width * this->width) + (this->height + this->height))) / 12; // +??
    }
    else {
        radius = this->height / 2.0f;
        this->body.setOrigin((float)radius, (float)radius);
        this->Inertia = (this->mass * (radius * radius)) / 2.0;
        //FROM WIKIPEDIA
    }
    this->body.setSize({(float)width, (float)height});
    this->body.setTexture(&texture);
    this->x = sf::Vector3<double>(posX, posY, 0.0f);
    this->body.setPosition((float)posX, (float)posY);
    this->P = sf::Vector3<double>(0,0,0);
    this->L = 0.0;
    this->angular_acceleration = 0.0;
    this->w = 0.0f;
    this->collision_found = false;
    this->fixed = fixed;
    //vector_texture.loadFromFile("Users/laurapessl/Desktop/Magnity/res/arrow.png");
    //this->momentum_vector.setTexture(vector_texture);
}

void RigidBody::DisplayBodies(sf::RenderWindow &window, std::vector<RigidBody*> *rigid_bodies) {
    //loop through all bodies and display them
    for (auto & rigid_bodie : *rigid_bodies) {
        if(rigid_bodie->visible) {
            window.draw(rigid_bodie->body);
        }
    }
}

double RigidBody::calcMagnitude(sf::Vector3<double> vec) {
    return std::sqrt((vec.x * vec.x) + (vec.y * vec.y));
}

sf::Vector3<double> RigidBody::calcCrossProd(sf::Vector3<double> vec1, sf::Vector3<double> vec2) {
    return {0.0f, 0.0f, (vec1.x * vec2.y) - (vec1.y * vec2.x)};
}


void RigidBody::ComputeForceAndTorque(RigidBody *rb, std::vector<RigidBody*> *rigid_bodies) {
    //compute all forces
    //TODO: check where player magnates are and compute force accordingly
    if(rb->type < 2) {
        //constants taken from Wikipedia
        double g = 9.8;
        double G = 6.67430e-11;
        double massA = rb->mass;
        sf::Vector3<double> gravity = {0.0, massA * g, 0.0};
        // sf::Vector3<double> gravity = {0.0, 0.0, 0.0};
        rb->force = {0.0, 0.0, 0.0};

        for (RigidBody* body : *rigid_bodies) {
            if (rb->id != body->id && body->type < 2) {
                // gravitational softening length
                #define EPSILON (1.0f/1000000)

                double massB = body->mass;
                double distance = std::sqrt( std::pow(body->x.x - rb->x.x, 2) + std::pow(body->x.y - rb->x.y, 2) ) / 100000000.f;

                //std::cout << "Distance = " << distance << std::endl;
                sf::Vector3<double> direction = rb->normalizeVector({body->x.x - rb->x.x, body->x.y - rb->x.y, 0.0});
                //std::cout << "Direction  (" << direction.x << ", " << direction.y << ")" << std::endl;
                double forceMagnitude = (G * massA * massB) / ((distance * distance) + pow(EPSILON, 2));
                //std::cout << "Force magnitude = " << forceMagnitude << std::endl;
                sf::Vector3<double> gravitationalForce = {forceMagnitude * direction.x, forceMagnitude * direction.y, 0.0};
                //std::cout << "gravitaional Force  (" << gravitationalForce.x << ", " << gravitationalForce.y << ")" << std::endl;

                // Apply gravitational force to rb and body
                rb->force = {rb->force.x + gravitationalForce.x, rb->force.y + gravitationalForce.y, 0.0};
                //std::cout << "Force in loop   (" << rb->force.x << ", " << rb->force.y << ")" << std::endl;
            }
        }

        rb->force = {rb->force.x + (gravity.x * 20), rb->force.y + (gravity.y * 20), 0.0}; //times 100 because its is in meters
        rb->force.y *= (rb->type == 1) ? -1 : 1;
//        std::cout << "Force   (" << rb->force.x << ", " << rb->force.y << ")" << std::endl;
    }
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

void RigidBody::applyVelocityVerletIntegration(RigidBody* rigid_body0, RigidBody* rigid_body1, double timestep, std::vector<RigidBody*> *rigid_bodies) {
    //1. Update Position
    sf::Vector3<double> tmp_velocity = sf::Vector3<double>(rigid_body0->v.x * timestep, rigid_body0->v.y * timestep, rigid_body0->v.z * timestep);

    double timestep2 = (timestep * timestep);

    sf::Vector3<double> tmp = sf::Vector3<double>(0.5 * rigid_body0->linear_acceleration.x * timestep2, 0.5 * rigid_body0->linear_acceleration.y * timestep2,
                                    0.5 * rigid_body0->linear_acceleration.z * timestep2);

    //get next position of rigid body
    rigid_body1->x = rigid_body0->x + sf::Vector3<double>(tmp_velocity.x + tmp.x, tmp_velocity.y + tmp.y, tmp_velocity.z + tmp.z);

    //calculate forces, the NOW act on rigid body
    ComputeForceAndTorque(rigid_body1, rigid_bodies);

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
void RigidBody::ode(std::vector<RigidBody*> *y0, std::vector<RigidBody> *yEnd, double t0, double t1,
                                       std::vector<RigidBody*>* rigid_bodies,
                                        std::vector<RigidBody*> *insertedBodies) {
    //Compute current state of object
    double timestep = t1 - t0;

    for(int i = 0; i < y0->size(); i++) {
        if(y0->at(i)->disabled || y0->at(i)->fixed || y0->at(i)->type == 2 || y0->at(i)->type == 3) {
            continue;
        }
        //std::cout << "Old position = " << y0->at(i)->x.x << ", " << y0->at(i)->x.y << ")" << std::endl;
        //STEPS
        //1. Use Velocity Verlet Integration to get new Position/Acceleration
        // apply forces and update linear

        applyVelocityVerletIntegration(y0->at(i), &yEnd->at(i), timestep, rigid_bodies);

        yEnd->at(i).P = {y0->at(i)->mass * y0->at(i)->v.x, y0->at(i)->mass * y0->at(i)->v.y, 0.0};


        //rigid bodies were updated -> now check for collisions at new position
        if(!y0->at(i)->collision_found) {
            y0->at(i)->checkForCollisions(rigid_bodies, insertedBodies);
            y0->at(i)->collision_found = false;
        }
        else {
            y0->at(i)->collision_found = false;
        }

        applyVelocityVerletIntegration(y0->at(i), &yEnd->at(i), timestep, rigid_bodies);

        yEnd->at(i).P = {y0->at(i)->mass * y0->at(i)->v.x, y0->at(i)->mass * y0->at(i)->v.y, 0.0};

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
        y0->at(i)->obj.position_ = sf::Vector2f(yEnd->at(i).x.x, yEnd->at(i).x.y);
        y0->at(i)->torque_vec = yEnd->at(i).torque_vec;
        y0->at(i)->force_points = yEnd->at(i).force_points;
        y0->at(i)->Inertia = yEnd->at(i).Inertia;
        y0->at(i)->P = yEnd->at(i).P;
        y0->at(i)->angular_acceleration = yEnd->at(i).angular_acceleration;
        y0->at(i)->w = yEnd->at(i).w;
        y0->at(i)->force = yEnd->at(i).force;
        y0->at(i)->linear_acceleration = yEnd->at(i).linear_acceleration;
        y0->at(i)->v = yEnd->at(i).v;
        y0->at(i)->L = yEnd->at(i).L;

    }

}

sf::Vector3<double> RigidBody::normalizeVector(sf::Vector3<double> vec) {
    if(calcMagnitude(vec) <= 0.0001) {
        return {0.0f, 0.0f, 0.0f};
    }

    return {(double)(vec.x / calcMagnitude(vec)), (double)(vec.y / calcMagnitude(vec)),(double)(vec.z / calcMagnitude(vec))};
}

void RigidBody::checkForCollisions(std::vector<RigidBody*> *rigid_bodies, std::vector<RigidBody*> *insertedBodies) {
    //go through all objects and check for collisions
    for (int i = 0; i < rigid_bodies->size(); i++) {
        
        if(rigid_bodies->at(i)->collision_found || rigid_bodies->at(i)->splitter || rigid_bodies->at(i)->disabled || rigid_bodies->at(i)->fixed) {
            continue;
        }

        //check if object is touching other rigid body
        for (int j = 0; j < rigid_bodies->size(); j++) {
            if (j == i || rigid_bodies->at(j)->splitter || rigid_bodies->at(j)->disabled || rigid_bodies->at(j)->fixed) {
                continue;
            }

            RigidBody* rb1 = rigid_bodies->at(i);
            RigidBody* rb2 = rigid_bodies->at(j);
            if ((rb1->type == 2 || rb1->type == 3) && (rb2->type == 2 || rb2->type == 3)) continue;

            double distance = 1000000000.0;
            double distanceObj = 0.0f;
            sf::Vector3<double> cOM_obstacle;
            sf::Vector3<double> collision_point;
            bool collision = false;


            if(rigid_bodies->at(j)->type == 2 || rigid_bodies->at(j)->type == 3) {
                distance = ::fabsf(rb1->x.y - rb2->x.y);
                distanceObj = (rb1->radius + (rb2->height / 2));
                double left_cornerX = rb2->x.x - (rb2->width / 2.0f);
                left_cornerX -= rb1->radius;
                double left_cornerY = rb2->x.y - (rb2->height / 2.0f);
                left_cornerY -= rb1->radius;
                double bottom_right_cornerX = rb2->x.x + (rb2->width / 2.0f);
                bottom_right_cornerX += rb1->radius;
                double bottom_right_cornerY = rb2->x.y + (rb2->height / 2.0f);
                bottom_right_cornerY += rb1->radius;

                if (rb1->x.x >= left_cornerX && rb1->x.x <= bottom_right_cornerX &&
                    rb1->x.y >= left_cornerY && rb1->x.y <= bottom_right_cornerY) {
                    collision = true;
                }
            }
            else {
                distance = std::sqrt(std::pow(rb2->x.x - rb1->x.x, 2) +
                                    std::pow(rb2->x.y - rb1->x.y, 2));
                distanceObj = rb2->radius + rb1->radius;
                if(distance <= distanceObj) {
                    collision = true;
                }
            }

            if (collision) {
                int ObstacleX = rb2->x.x;
                int ObstacleY = rb2->x.y;
                if(/* rb2->fixed && */rb2->type == 2 || rb2->type == 3) {
                    double left_cornerX = rigid_bodies->at(j)->x.x - (rigid_bodies->at(j)->width / 2.0f);
                    double left_cornerY = rigid_bodies->at(j)->x.y - (rigid_bodies->at(j)->height / 2.0f);
                    double bottom_right_cornerX = rigid_bodies->at(j)->x.x + (rigid_bodies->at(j)->width / 2.0f);
                    double bottom_right_cornerY = rigid_bodies->at(j)->x.y + (rigid_bodies->at(j)->height / 2.0f);

                    if(rb1->x.y <= left_cornerY) { // Above
                        rb1->x.y = rigid_bodies->at(j)->x.y - (rigid_bodies->at(j)->height / 2.0) - rb1->radius;
                        collision_point = sf::Vector3<double>(rb1->x.x, rb1->x.y + rb1->radius, 0.0f);
                        rigid_bodies->at(j)->x.x = collision_point.x;
                    } else if(rb1->x.y >= bottom_right_cornerY) { //Beneath
                        rb1->x.y = rigid_bodies->at(j)->x.y + (rigid_bodies->at(j)->height / 2.0) + rb1->radius;
                        collision_point = sf::Vector3<double>(rb1->x.x, rb1->x.y - rb1->radius, 0.0f);
                        rigid_bodies->at(j)->x.x = collision_point.x;
                    } else {
                        if(rb1->x.x <= left_cornerX) { //Left
                            rb1->x.x = rigid_bodies->at(j)->x.x - (rigid_bodies->at(j)->width / 2.0) - rb1->radius;
                            collision_point = sf::Vector3<double>(rb1->x.x + rb1->radius, rb1->x.y, 0.0f);
                            rigid_bodies->at(j)->x.y = collision_point.y;
                        } else { //Right
                            rb1->x.x = rigid_bodies->at(j)->x.x + (rigid_bodies->at(j)->width / 2.0) + rb1->radius;
                            collision_point = sf::Vector3<double>(rb1->x.x - rb1->radius, rb1->x.y, 0.0f);
                            rigid_bodies->at(j)->x.y = collision_point.y;
                        }
                    }

                    rb2->v = {0,0,0};
                    rb2->torque_vec = {0,0,0};
                    rb2->L = 0.0f;
                    applyCollision(rb1, rb2, collision_point);
                    rb2->v = {0,0,0};
                    rb2->torque_vec = {0,0,0};
                    rb2->L = 0.0f;
                    rb2->x.x = ObstacleX;
                    rb2->x.y = ObstacleY;
                    rb2->collision_found = false;
                }
                else {
                    sf::Vector3<double> VNorm = normalizeVector(rb1->v);
                    sf::Vector3<double> displacement = {-VNorm.x * (distanceObj - distance), -VNorm.y * (distanceObj - distance), 0.0f};

                    sf::Vector3<double> newCenter = {rb1->x.x + displacement.x,
                                                     rb1->x.y + displacement.y, 0.0f};

                    rb1->x = newCenter;

                    //sleep(5);

                    collision_point = (rb1->x * rb2->radius +
                                                           rb2->x * rb1->radius) /
                                                          (rb1->radius + rb2->radius);

                    applyCollision(rb1, rb2, collision_point);
                }
                if (rb2->type == 3)
                {
                    VoronoiFracture(rb1, collision_point).calcualteVoronoiFracture(insertedBodies);
                    if (in_game)
                    {
                        lost = true;
                    }
                    // game_paused = true;
                }
            }
        }
    }
}

void RigidBody::applyCollision(RigidBody *rigidBody1, RigidBody *rigidBody2, sf::Vector3<double> collision_point) {
    //calculate normal
    sf::Vector3<double> normal = normalizeVector(rigidBody1->x - collision_point);

    sf::Vector3<double> normal2 = sf::Vector3<double>(normal.x * normal.x, normal.y * normal.y,
                                                      normal.z * normal.z);
    //calculate impulse -> new velocities
    sf::Vector3<double> v1_v2 = rigidBody1->v - rigidBody2->v;

    double divMass1 =
            (2 * rigidBody2->mass) / (rigidBody1->mass + rigidBody2->mass);
    double divMass2 =
            (2 * rigidBody1->mass) / (rigidBody1->mass + rigidBody2->mass);


    sf::Vector3<double> right_side1 = sf::Vector3<double>(v1_v2.x * divMass1, v1_v2.y * divMass1,
                                                          v1_v2.z * divMass1);
    right_side1 = sf::Vector3<double>(right_side1.x * normal2.x, right_side1.y * normal2.y,
                                      right_side1.z * normal2.z);

    sf::Vector3<double> right_side2 = sf::Vector3<double>(v1_v2.x * divMass2, v1_v2.y * divMass2,
                                                          v1_v2.z * divMass2);
    right_side2 = sf::Vector3<double>(right_side2.x * normal2.x, right_side2.y * normal2.y,
                                      right_side2.z * normal2.z);

    //calculate J = j * n for Torque
    sf::Vector3<double> r1 = collision_point - rigidBody1->x;
    sf::Vector3<double> r2 = collision_point - rigidBody2->x;

    //-------------------------------------- from old code -----------------------------------------
    sf::Vector3<double> p1 = rigidBody1->v + calcCrossProd(r1, sf::Vector3<double>(0.0, 0.0, rigidBody1->w));
    sf::Vector3<double> p2 = rigidBody2->v + calcCrossProd(r2, sf::Vector3<double>(0.0, 0.0, rigidBody2->w));

    sf::Vector3<double> p1_p2 = p1 - p2;

    double dividend = -2 * ( (p1_p2.x * normal.x) + (p1_p2.y * normal.y) + (p1_p2.z * normal.z) );
    double divisor = std::pow(rigidBody1->mass, -1) + std::pow(rigidBody2->mass, -1);

    double j_imp = dividend / divisor;

    sf::Vector3<double> J = sf::Vector3<double>(normal.x * j_imp, normal.y * j_imp, normal.z * j_imp);

    rigidBody1->v = rigidBody1->v + J * (1.0f / rigidBody1->mass);;
    rigidBody2->v = rigidBody2->v - J * (1.0f / rigidBody2->mass);;

    //set torque vector
    rigidBody1->torque_vec = calcCrossProd(r1, right_side1);
    rigidBody2->torque_vec = calcCrossProd(r2, -right_side2);

    //update angular momentum
    rigidBody1->L += rigidBody1->torque_vec.z;
    rigidBody2->L += rigidBody2->torque_vec.z;

    rigidBody2->collision_found = true;
    rigidBody1->collision_found = true;
}

void RigidBody::updateRigidBodies(std::vector<RigidBody *> *rigid_bodies, float total_time, float deltaTime) {

    std::vector<RigidBody> *rigid_bodies_new = new std::vector<RigidBody>;
    //Run Rigid Body simulation
    for(int i = 0; i < rigid_bodies->size(); i++) {
        rigid_bodies_new->push_back(*rigid_bodies->at(i));
    }


    total_time += deltaTime;
    std::vector<RigidBody*> deleteBodies;
    std::vector<RigidBody*> *insertedBodies = new std::vector<RigidBody*>;
    RigidBody::ode(rigid_bodies, rigid_bodies_new,total_time - deltaTime,
                   total_time, rigid_bodies, insertedBodies);


    //loop through bodies and delete or insert bodies
    for(int i = 0; i < insertedBodies->size(); i++) {
        insertedBodies->at(i)->id = insertedBodies->at(i)->id + rigid_bodies->size();
        rigid_bodies->push_back(insertedBodies->at(i));
    }

    insertedBodies->clear();
    delete insertedBodies;

    //delete rigid_bodies_new we don't need it anymore
    rigid_bodies_new->clear();
    delete rigid_bodies_new;
}

void RigidBody::drawVelocityArrows(sf::RenderWindow& window, std::vector<RigidBody*> *rigid_bodies)
{
    if (!RigidBody::draw_vectors_) return;

    sf::Texture arrow_tex;
    arrow_tex.loadFromFile("res/arrow_red.png");

    for (int x = 0; x < rigid_bodies->size(); x++)
    {
        if (!rigid_bodies->at(x)->visible) continue;

        sf::Vector3<double> norm = rigid_bodies->at(x)->normalizeVector(rigid_bodies->at(x)->P);
        sf::Vector2f pos = {(float)rigid_bodies->at(x)->x.x, (float)rigid_bodies->at(x)->x.y};
        sf::Vector2f pos_angular = {(float)rigid_bodies->at(x)->x.x, (float)rigid_bodies->at(x)->x.y - (float)rigid_bodies->at(x)->radius - 2.0f};
        sf::Vector2f linear_momentum = {(float)norm.x, (float)norm.y};
        
        rigid_bodies->at(x)->drawArrow(window, arrow_tex, pos, linear_momentum);
        rigid_bodies->at(x)->drawAngularMomentum(window, pos, rigid_bodies->at(x)->L);
    }
}

void RigidBody::drawArrow(sf::RenderWindow& window, sf::Texture& tex, sf::Vector2f position, sf::Vector2f P)
{
    sf::Vector2f arrowEnd = {position.x + (P.x * 10), position.y + (P.y * 10)};
    float orientation = atan2(P.y, P.x) * 180.0f / 3.14159f;

    SpriteObject o(tex, {0, 0}, 0);
    o.setScale({0.05f, 0.05f});
    o.setPosition(arrowEnd);
    o.setRotation(orientation);

    o.draw(window);
}

void RigidBody::drawAngularMomentum(sf::RenderWindow& window, sf::Vector2f position, double L)
{
    sf::Text angular_momentum;
    sf::Font f;
    f.loadFromFile("res/default_font.ttf");
    angular_momentum.setFont(f);
    angular_momentum.setCharacterSize(10);
    angular_momentum.setOrigin(80, -20);
    angular_momentum.setFillColor(sf::Color::Black);
    std::string text = "Angular Momentum = " + std::to_string(L);
    angular_momentum.setString(text);
    angular_momentum.setPosition(position.x, position.y);

    window.draw(angular_momentum);
}

