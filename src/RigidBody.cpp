#include "RigidBody.h"
#include "Contact.h"
#include <iostream>
#include <SFML/Graphics.hpp>
#include <unistd.h>

#define M_PI 3.14159265358979323846
#define NUM_RIGID_BODIES 2
//array of rigid bodies
std::vector<RigidBody*>* rigid_bodies = new std::vector<RigidBody*>;
unsigned int highest_id = 0;
std::vector<Contact*>* contacts = new std::vector<Contact*>;
int ncontacts = 0;
int count_contacts = 0;

RigidBody::RigidBody(float mass, float density, unsigned int type, float width, float height, const sf::Texture& texture,
                     bool fixed, float posX, float posY) {
    this->type = type;
    this->fixed = fixed;
    this->height = height;
    this->width = width;
    if(type == 0) {
        radius = this->height / 2.0f;
        this->body.setOrigin(radius, radius);
    }
    else {
        this->body.setOrigin(width / 2.0f, height / 2.0f);
    }
    this->body.setSize({height, width});
    this->body.setTexture(&texture);
    this->world_coord = sf::Vector3f(posX, posY, 0.0f);
    this->body.setPosition(this->world_coord.x, this->world_coord.y);
    highest_id++;
    this->mass = mass;
    this->Ibody = this->calcIbody();
    this->Ibodyinv = this->calcIinversebody();
    this->x = sf::Vector3f(0.0f, 0.0f,0);
    this->R = Matrix(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    this->P = sf::Vector3f(0,0,0);
    this->L = sf::Vector3f(0,0,0);
    this->omega = sf::Vector3f(0,0,0);
    rigid_bodies->push_back(this);
}

void RigidBody::DisplayBodies(sf::RenderWindow &window) {
    //loop through all bodies and display them
    for (auto & rigid_bodie : *rigid_bodies) {
        window.draw(rigid_bodie->body);
    }
}

Matrix RigidBody::calcMatrixMult(Matrix matrix1, Matrix matrix2) {
    double aa = (matrix2.aa * matrix1.aa) + (matrix2.ba * matrix1.ab) + (matrix2.ca * matrix1.ac);
    double ab = (matrix2.ab * matrix1.aa) + (matrix2.bb * matrix1.ab) + (matrix2.cb * matrix1.ac);
    double ac = (matrix2.ca * matrix1.aa) + (matrix2.cb * matrix1.ab) + (matrix2.cc * matrix1.ac);
    double ba = (matrix2.aa * matrix1.ba) + (matrix2.ba * matrix1.bb) + (matrix2.ca * matrix1.bc);
    double bb = (matrix2.ab * matrix1.ba) + (matrix2.bb * matrix1.bb) + (matrix2.cb * matrix1.bc);
    double bc = (matrix2.ca * matrix1.ba) + (matrix2.cb * matrix1.bb) + (matrix2.cc * matrix1.bc);
    double ca = (matrix2.aa * matrix1.ca) + (matrix2.ba * matrix1.cb) + (matrix2.ca * matrix1.cc);
    double cb = (matrix2.ab * matrix1.ca) + (matrix2.bb * matrix1.cb) + (matrix2.cb * matrix1.cc);
    double cc = (matrix2.ca * matrix1.ca) + (matrix2.cb * matrix1.cb) + (matrix2.cc * matrix1.cc);

    return {aa, ab, ac, ba, bb, bc, ca, cb, cc};

}

sf::Vector3f RigidBody::calcMatrixMult(Matrix matrix1, sf::Vector3f vec) {
    double a = (matrix1.aa * vec.x) + (matrix1.ab * vec.y) + (matrix1.ac * vec.z);
    double b = (matrix1.ba * vec.x) + (matrix1.bb * vec.y) + (matrix1.bc * vec.z);
    double c = (matrix1.ca * vec.x) + (matrix1.cb * vec.y) + (matrix1.cc * vec.z);

    return {(float)a,(float)b,(float)c};
}

double RigidBody::calcMagnitude(sf::Vector3f vec) {
    return std::sqrt((vec.x * vec.x) + (vec.y * vec.y));
}

double RigidBody::calcDotProd(sf::Vector3f vec1, sf::Vector3f vec2) {
    return (vec1.x * vec2.x) + (vec1.y * vec2.y);
}

sf::Vector3f RigidBody::calcCrossProd(sf::Vector3f vec1, sf::Vector3f vec2) {
    return {0.0f, 0.0f, (vec1.x * vec2.y) - (vec1.y * vec2.x)};
}

sf::Vector3f RigidBody::calcDivScalar(sf::Vector3f vec, double scalar) {
    return {(float)(vec.x / scalar), (float)(vec.y / scalar),(float)(vec.z / scalar)};
}

void RigidBody::StateToArray(RigidBody *rb, double *y) {
    *y++ = rb->x.x;
    *y++ = rb->x.y;
    *y++ = rb->x.z;

    *y++ = rb->q.r;
    *y++ = rb->q.i;
    *y++ = rb->q.j;
    *y++ = rb->q.k;

    *y++ = rb->P.x;
    *y++ = rb->P.y;
    *y++ = rb->P.z;

    *y++ = rb->L.x;
    *y++ = rb->L.y;
    *y++ = rb->L.z;
}

void RigidBody::ArrayToState(RigidBody *rb, double *y) {

    rb->x.x = (float)*y++;
    rb->x.y = (float)*y++;
    rb->x.z = (float)*y++;

    //Assume that a quaternion is represented in
    //terms of elements 'r' for the real part
    //and 'i', 'j' and 'k' for the vector part
    *y++ = rb->q.r;
    *y++ = rb->q.i;
    *y++ = rb->q.j;
    *y++ = rb->q.k;

    rb->P.x = (float)*y++;
    rb->P.y = (float)*y++;
    rb->P.z = (float)*y++;

    rb->L.x = (float)*y++;
    rb->L.y = (float)*y++;
    rb->L.z = (float)*y++;

    //Compute auxiliary variables
    rb->v = calcDivScalar(rb->P, rb->mass);
    rb->R = QuaternionToMatrix(normalizeQuant(rb->q));


    rb->Iinv = calcMatrixMult(calcMatrixMult(rb->R, rb->Ibodyinv), calcTransponseMatrix(rb->R));
    rb->omega = calcMatrixMult(rb->Iinv, rb->L);
}

void RigidBody::ArrayToBodies(double *x) {
    for(int i = 0; i < rigid_bodies->size(); i++) {
        ArrayToState(rigid_bodies->at(i), &x[i * STATE_SIZE]);
    }
}

void RigidBody::BodiesToArray(double *x) {
    for(int i = 0; i < rigid_bodies->size(); i++) {
        StateToArray(rigid_bodies->at(i), &x[i * STATE_SIZE]);
    }
}

void RigidBody::Dxdt(double t, double x[], double xdot[]) {
    //put data in x[] into rigid_bodies vector
    ArrayToBodies(x);

    for(int i = 0; i < rigid_bodies->size(); i++) {
        ComputeForceAndTorque(t, rigid_bodies->at(i));
        DdtStateToArray(rigid_bodies->at(i), &xdot[i * STATE_SIZE]);
    }
}

void RigidBody::DdtStateToArray(RigidBody *rb, double *xdot) {
    //copy v(t) into xdot
    *xdot++ = rb->v.x;
    *xdot++ = rb->v.y;
    *xdot++ = rb->v.z;


    Quaternion qdot = calcQuatMult(calcQuatMult(Quaternion(0, rb->omega), rb->q), 0.5f);
    *xdot++ = qdot.r;
    *xdot++ = qdot.i;
    *xdot++ = qdot.j;
    *xdot++ = qdot.k;

    *xdot++ = rb->force.x;
    *xdot++ = rb->force.y;
    *xdot++ = rb->force.z;

    *xdot++ = rb->torque.x;
    *xdot++ = rb->torque.y;
    *xdot++ = rb->torque.z;
}

Matrix RigidBody::calcTransponseMatrix(Matrix matrix) {
    return Matrix(matrix.aa, matrix.ba, matrix.ca, matrix.ab, matrix.bb, matrix.cb, matrix.ac, matrix.bc, matrix.cc);
}

void RigidBody::ComputeForceAndTorque(double t, RigidBody *rb) {
    if(rb->type == 1) {
        rb->force = sf::Vector3f(rb->force.x, -9.81f * rb->mass, rb->force.z);
    }
    else {
        rb->force = sf::Vector3f(rb->force.x, 9.81f * rb->mass, rb->force.z);
    }
    //store torque in rb->torque
    //store sum of forces in rb->force
    //TODO: function that checks where magnets are and compute force that they have on object
    //TODO: add all forces together!!
    //Total force F(t) = Sum of all forces Fi(t)
    //Total external torque = Sum of( (ri(t) - x(t)) x Fi(t) ) -> volume times total force
    //Gravity = 9.81f * mass

    sf::Vector3f distanceToCoM = sf::Vector3f(rb->width / 2.0f, rb->height / 2.0f, 0.0f);
    //torque = r.x * f.y - r.y * f.x;
    rb->torque = calcCrossProd(distanceToCoM, rb->force);
    if(ncontacts != 0) {
        //TODO: loop through contacts
        contacts->at(0)->DetectAndResolveCollision(*contacts->at(0));

        /*std::cout << "Object A  L    X: " << contacts->at(0)->a->L.x << " Y: " << contacts->at(0)->a->L.y << std::endl;
        std::cout << "Object B  L    X: " << contacts->at(0)->b->L.x << " Y: " << contacts->at(0)->b->L.y << std::endl;
        std::cout << "Object A  P    X: " << contacts->at(0)->a->P.x << " Y: " << contacts->at(0)->a->P.y << std::endl;
        std::cout << "Object B  P    X: " << contacts->at(0)->b->P.x << " Y: " << contacts->at(0)->b->P.y << std::endl;
        std::cout << "Object A  Force    X: " << contacts->at(0)->a->force.x << " Y: " << contacts->at(0)->a->force.y << std::endl;
        std::cout << "Object B  Force    X: " << contacts->at(0)->b->force.x << " Y: " << contacts->at(0)->b->force.y << std::endl;*/
        ncontacts = 0;
        contacts->clear();
    }

}

Quaternion RigidBody::normalizeQuant(Quaternion quaternion) {
    double length = calcMagnitude(sf::Vector3f(quaternion.i, quaternion.j, quaternion.k));
    if(length == 0) {
        return {0.0f, 0.0f, 0.0f, 0.0f};
    }
    sf::Vector3f vec = calcDivScalar(sf::Vector3f(quaternion.i, quaternion.j, quaternion.k), length);
    return {quaternion.r, vec.x, vec.y, vec.z};
}

Matrix RigidBody::QuaternionToMatrix(Quaternion quaternion) {
    double aa = 1 - (2 * (quaternion.j * quaternion.j)) - (2 * (quaternion.k * quaternion.k));
    double ab = (2 * quaternion.i * quaternion.j) - (2 * quaternion.r * quaternion.k);
    double ac = (2 * quaternion.i * quaternion.k) + (2 * quaternion.r * quaternion.i);
    double ba = (2 * quaternion.i * quaternion.j) + (2 * quaternion.r * quaternion.k);
    double bb = 1 - (2 * (quaternion.i * quaternion.i)) - (2 * (quaternion.k * quaternion.k));
    double bc = (2 * quaternion.j * quaternion.k) - (2 * quaternion.r * quaternion.i);
    double ca = (2 * quaternion.i * quaternion.k) - (2 * quaternion.r * quaternion.j);
    double cb = (2 * quaternion.j * quaternion.k) + (2 * quaternion.r * quaternion.i);
    double cc = 1 - (2 * (quaternion.i * quaternion.i)) - (2 * (quaternion.j * quaternion.j));
    return {aa, ab, ac, ba, bb, bc, ca, cb, cc};
}

Quaternion RigidBody::calcQuatMult(Quaternion q1, Quaternion q2) {
    sf::Vector3f v1 = sf::Vector3f(q1.i, q1.j, q1.k);
    sf::Vector3f v2 = sf::Vector3f(q2.i, q2.j, q2.k);

    Quaternion res;
    res.r = (q1.r * q2.r) - (calcDotProd(v1, v2));
    sf::Vector3f s1v2 = sf::Vector3f(q1.r * v2.x, q1.r * v2.y, q1.r * v2.z);
    sf::Vector3f s2v1 = sf::Vector3f(q2.r * v1.x, q2.r * v1.y, q2.r * v1.z);
    sf::Vector3f tmp1 = sf::Vector3f(s1v2.x + s2v1.x, s1v2.y + s2v1.y, s1v2.z + s2v1.z);
    sf::Vector3f tmp2 = calcCrossProd(v1, v2);
    sf::Vector3f v = sf::Vector3f(tmp1.x + tmp2.x, tmp1.y + tmp2.y, tmp1.z + tmp2.z);
    res.i = v.x;
    res.j = v.y;
    res.k = v.z;

    return res;
}

//TODO: idk if this is correct
Quaternion RigidBody::calcQuatMult(Quaternion q, double scalar) {
    return {q.r * scalar, q.i * scalar, q.j * scalar, q.k * scalar};
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

Matrix RigidBody::calcIinversebody() {
    //calc determinate
    double det = (this->Ibody.aa * this->Ibody.bb * this->Ibody.cc) +
                 (this->Ibody.ab * this->Ibody.bc * this->Ibody.ca) +
                 (this->Ibody.ac * this->Ibody.ba * this->Ibody.cb) -
                 (this->Ibody.ac * this->Ibody.bb * this->Ibody.ca) -
                 (this->Ibody.aa * this->Ibody.bc * this->Ibody.cb) -
                 (this->Ibody.ab * this->Ibody.ba * this->Ibody.cc);

    if(det == 0.0f) {
        //TODO: should I here return this->Ibody?
        return {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        //return this->Ibody;
    }
    else {
        Matrix T = calcTransponseMatrix(this->Ibody);
        Matrix tmp = Matrix(
                (this->Ibody.bb * this->Ibody.cc) - (this->Ibody.bc * this->Ibody.cb),
                (this->Ibody.ba * this->Ibody.cc) - (this->Ibody.bc * this->Ibody.ca),
                (this->Ibody.ba * this->Ibody.cb) - (this->Ibody.bb * this->Ibody.ca),
                (this->Ibody.ab * this->Ibody.cc) - (this->Ibody.ac * this->Ibody.cb),
                (this->Ibody.aa * this->Ibody.cc) - (this->Ibody.ac * this->Ibody.ca),
                (this->Ibody.aa * this->Ibody.cb) - (this->Ibody.ab * this->Ibody.ca),
                (this->Ibody.ab * this->Ibody.bc) - (this->Ibody.ac * this->Ibody.bb),
                (this->Ibody.aa * this->Ibody.bc) - (this->Ibody.ac * this->Ibody.ba),
                (this->Ibody.aa * this->Ibody.bb) - (this->Ibody.ab * this->Ibody.ba)
        );

        Matrix tmp1 = Matrix(1.0f, -1.0f, 1.0f, -1.0f, 1.0f, -1.0f, 1.0f, -1.0f, 1.0f);
        Matrix adj = calcMatrixMult(tmp, tmp1);

        if((1/det) == 1.0f) {
            return adj;
        }
        else {
            return {adj.aa * (1/det), adj.ab * (1/det), adj.ac * (1/det), adj.ba * (1/det), adj.bb * (1/det),
                    adj.bc * (1/det), adj.ca * (1/det), adj.cb * (1/det), adj.cc * (1/det)};
        }
    }
}

float t = 0;
void RigidBody::RunSimulation(float deltaTime, sf::RenderWindow &window)
{
    double x0[STATE_SIZE * rigid_bodies->size()], xFinal[STATE_SIZE * rigid_bodies->size()];
    BodiesToArray(xFinal);

    t += deltaTime;
    for(int i = 0; i < STATE_SIZE * rigid_bodies->size(); i++) {
        x0[i] = xFinal[i];
    }

    ode(x0, xFinal, STATE_SIZE * rigid_bodies->size(), t-deltaTime, t);
    ArrayToBodies(xFinal);
    checkForCollisions();

    for (auto & rigid_bodie : *rigid_bodies) {
        rigid_bodie->world_coord.x = rigid_bodie->world_coord.x + rigid_bodie->x.x;
        rigid_bodie->world_coord.y = rigid_bodie->world_coord.y + rigid_bodie->x.y;
        rigid_bodie->body.setPosition(rigid_bodie->world_coord.x, rigid_bodie->world_coord.y);
    }
}

//TODO: This is from the sheet
//RIGID BODY SHEET
typedef void (*DerivFunc)(double t, double x[], double xdot[]);

//x0 = init state vector
//len = len of x0
//t0 = starting time
//t1 = end time
//solver needs to compute the state vector at time t1 and return it in xEnd
//function dxdt -> has array x that encodes state vector x(t), it must returnd d/dt x(t) in the xdot
//we need t in dxdt because we may have time varying forces
//ode can call dxdt as often as it likes
void RigidBody::ode(double x0[], double xEnd[], int len, double t0, double t1) {

    double timestep = t1 - t0;
    double x_half[len], v_half[len];

    // Compute initial velocity
    double v0[len], k1[len];
    rigid_bodies->at(0)->Dxdt(t0, x0, k1);
    for (int i = 0; i < len; i++) {
        v0[i] = x0[i] + 0.5 * timestep * k1[i];
    }

    // Update position and velocity
    for (int i = 0; i < len; i++) {
        x_half[i] = x0[i] + timestep * v0[i];
    }
    rigid_bodies->at(0)->Dxdt(t1, x_half, k1);

    for (int i = 0; i < len; i++) {
        v_half[i] = v0[i] + 0.5 * timestep * k1[i];
    }
    for (int i = 0; i < len; i++) {
        xEnd[i] = x_half[i] + timestep * v_half[i];
    }
}

sf::Vector3f RigidBody::normalizeVector(sf::Vector3f vec) {
    if(calcMagnitude(vec) == 0) {
        return {0.0f, 0.0f, 0.0f};
    }

    return {(float)(vec.x / calcMagnitude(vec)), (float)(vec.y / calcMagnitude(vec)),(float)(vec.z / calcMagnitude(vec))};
}

void RigidBody::checkForCollisions() {
    //go through all objects
    //for(int i = 0;  i < NBODIES; i++) {
    //calculate bounding box length of object
    double umfang = 0.0f;
    if(rigid_bodies->at(0)->type == 0) {
        umfang = 2 * M_PI * rigid_bodies->at(0)->radius;
    }

    //send out vectors (rays) out from the center of mass and check
    //if it intersects with another object
    int numberPoints = round(umfang * 1.5f); //we want to check every 0.5 steps on the circle
    for(double j = 0; j < numberPoints; j++) {
        float angle = 0.5f * j;
        float x = rigid_bodies->at(0)->radius * std::cos(angle);
        float y = rigid_bodies->at(0)->radius * std::sin(angle);

        //new point where the vector is pointing to
        //add some epsilon to it
        sf::Vector3f newpoint = sf::Vector3f(x + rigid_bodies->at(0)->world_coord.x + 5,
                                             y + rigid_bodies->at(0)->world_coord.y + 5, 0.0f);

        //TODO: now loop through all other objects and check if object lies on vector
        //for now use Border
        //sf::Vector3f border_com = {400.0f, 350.0f, 0.0f};
        //loop through upper side of border and check if there is intersection
        //check if this point is somewhere near calculated other point + in our case 5 because height of border = 5
        sf::Vector3f vector = newpoint - rigid_bodies->at(0)->world_coord;
        sf::Vector3f zToStart = rigid_bodies->at(0)->world_coord - rigid_bodies->at(1)->world_coord;
        float dotProduct = vector.x * zToStart.x + vector.y * zToStart.y;

        if(newpoint.y >= ((rigid_bodies->at(1)->world_coord.y - rigid_bodies->at(1)->radius)) && zToStart.y <= 5.0f && zToStart.y >= -5.0f) {
            sf::Vector3f n = sf::Vector3f(0.0f, 1.0f, 0.0f);
            sf::Vector3f p = sf::Vector3f(rigid_bodies->at(1)->world_coord.x, newpoint.y - rigid_bodies->at(0)->radius, 0.0f); //world-space vertex location

            Contact *newcontact = new Contact(rigid_bodies->at(0), rigid_bodies->at(1), n, p);
            contacts->push_back(newcontact);
            ncontacts++;
            count_contacts++;
            rigid_bodies->at(0)->x.x = 0.0f;
            rigid_bodies->at(0)->x.y = 0.0f;
            return;
        }
    }
    //}
}