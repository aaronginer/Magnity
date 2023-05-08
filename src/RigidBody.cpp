#include "RigidBody.h"
#include <iostream>
#include <SFML/Graphics.hpp>

unsigned int highest_id = 0;
//array of rigid bodies
std::vector<RigidBody*>* rigid_bodies = new std::vector<RigidBody*>;
int ncontacts;
Contact *contacts;

RigidBody::RigidBody(float mass, float density, unsigned int type, float width, float height) {
    this->density = density;
    this->id = highest_id;
    highest_id++;
    this->type = type;
    this->height = height;
    this->width = width;
    if(type == 0) {
        radius = this->height / 2;
    }

    //// TODO Assign those values here
    //constant quantities
    this->mass = mass;
    this->Ibody = this->calcIbody();
    this->Ibodyinv = this->calcIinversebody();

    //those are fine, they will get changed
    this->x = sf::Vector3f(0.0f, 0.0f,0);
    this->R = Matrix(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    this->P = sf::Vector3f(0,0,0);
    this->L = sf::Vector3f(0,0,0);
}

void RigidBody::addToRigidBodies(RigidBody *rigidBody) {
    rigid_bodies->push_back(rigidBody);
}

//TODO: this does not make sense
void RigidBody::calcCenterOfMass() {
    //center of mass
    //in our 2D with equal density and rectangle it's the middle of the object
    if(this->type == 1) { //rectangle
        this->x.x = this->width / 2;
        this->x.y = this->height / 2;
        this->x.z = 0.0f;
    }
    else if(this->type == 0) { //circle
        this->x.x = radius;
        this->x.y = radius;
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

    return Matrix(aa, ab, ac, ba, bb, bc, ca, cb, cc);

}

sf::Vector3f RigidBody::calcMatrixMult(Matrix matrix1, sf::Vector3f vec) {
    double a = (matrix1.aa * vec.x) + (matrix1.ab * vec.y) + (matrix1.ac * vec.z);
    double b = (matrix1.ba * vec.x) + (matrix1.bb * vec.y) + (matrix1.bc * vec.z);
    double c = (matrix1.ca * vec.x) + (matrix1.cb * vec.y) + (matrix1.cc * vec.z);

    return sf::Vector3f(a,b,c);
}

sf::Vector3f RigidBody::getPosition() {
    return this->x;
}

unsigned int RigidBody::getType() {
    return this->type;
}

unsigned int RigidBody::getID() {
    return this->id;
}

double RigidBody::calcMagnitude(sf::Vector3f vec) {
    return std::sqrt((vec.x * vec.x) + (vec.y * vec.y));
}

sf::Vector3f RigidBody::calcUnitVector(sf::Vector3f vec) {
    return sf::Vector3f(vec.x / calcMagnitude(vec), vec.y / calcMagnitude(vec), 0.0f);
}

double RigidBody::calcDotProd(sf::Vector3f vec1, sf::Vector3f vec2) {
    return (vec1.x * vec2.x) + (vec1.y * vec1.y);
}

sf::Vector3f RigidBody::calcCrossProd(sf::Vector3f vec1, sf::Vector3f vec2) {
    return sf::Vector3f(0.0f, 0.0f, (vec1.x * vec2.y) - (vec1.y * vec2.x));
}

sf::Vector3f RigidBody::addVectors(sf::Vector3f vec1, sf::Vector3f vec2) {
    return sf::Vector3f(vec1.x + vec2.x, vec1.y + vec2.y, 0.0f);
}

sf::Vector3f RigidBody::multScalar(sf::Vector3f vec, double scalar) {
    return sf::Vector3f(vec.x * scalar, vec.y * scalar, 0.0f);
}

sf::Vector3f RigidBody::negateVector(sf::Vector3f vec) {
    return sf::Vector3f(vec.x * -1, vec.y * -1, 0.0f);
}

sf::Vector3f RigidBody::calcDivScalar(sf::Vector3f vec, double scalar) {
    return sf::Vector3f(vec.x / scalar, vec.y / scalar, vec.z / scalar);
}

double RigidBody::calcInertia(sf::Vector3f point_force, sf::Vector3f force) {
    if(this->type == 1) //is rectangle
    {
        return (this->mass * ((this->height * this->height) + (this->width * this->width))) / 12;
    }
    else //TODO: is circle
    {
        return (this->mass * ((this->height * this->height) + (this->width * this->width))) / 12;
    }
}

double RigidBody::calcTau(sf::Vector3f point_force, sf::Vector3f force) {
    return (point_force.x * force.x) - (point_force.y * force.y);
}

void RigidBody::StateToArray(RigidBody *rb, double *y) {
    *y++ = rb->x.x; //x component of position
    *y++ = rb->x.y; //y component of position
    *y++ = rb->x.z; //z component of position

    //Assume that a quaternion is represented in
    //terms of elements 'r' for the real part
    //and 'i', 'j' and 'k' for the vector part
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
    std::cout << "ArrayToState" << std::endl;
    rb->x.x = *y++;
    rb->x.y = *y++;
    rb->x.z = *y++;

    std::cout << "Center of mass: X " << rb->x.x << " Y " << rb->x.y << " Z: " << rb->x.z << std::endl;

    //Assume that a quaternion is represented in
    //terms of elements 'r' for the real part
    //and 'i', 'j' and 'k' for the vector part
    *y++ = rb->q.r;
    *y++ = rb->q.i;
    *y++ = rb->q.j;
    *y++ = rb->q.k;

    rb->P.x = *y++;
    rb->P.y = *y++;
    rb->P.z = *y++;

    rb->L.x = *y++;
    rb->L.y = *y++;
    rb->L.z = *y++;

    //Compute auxiliary variables
    rb->v = calcDivScalar(rb->P, this->mass);
    rb->R = QuaternionToMatrix(normalizeQuant(rb->q));
    rb->Iinv = calcMatrixMult(calcMatrixMult(R, Ibodyinv), calcTransponseMatrix(R));
    rb->omega = calcMatrixMult(rb->Iinv, rb->L);
}

void RigidBody::ArrayToBodies(double *x) {
    std::cout << "Array To Bodies" << std::endl;
    for(int i = 0; i < NBODIES; i++) {
        ArrayToState(rigid_bodies->at(i), &x[i * STATE_SIZE]);
    }
}

void RigidBody::BodiesToArray(double *x) {
    std::cout << "Bodies To Array" << std::endl;
    for(int i = 0; i < NBODIES; i++) {
        StateToArray(rigid_bodies->at(i), &x[i * STATE_SIZE]);
    }
}
void RigidBody::Dxdt(double t, double x[], double xdot[]) {
    //put data in x[] into rigid_bodies vector
    ArrayToBodies(x);

    for(int i = 0; i < NBODIES; i++) {
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

Matrix RigidBody::Star(sf::Vector3f vec) {
    return Matrix(0.0f, (vec.z * -1.0f), vec.y, vec.z, 0.0f, (vec.x * -1.0f), (vec.y * -1.0f), vec.x, 0.0f);
}

Matrix RigidBody::calcTransponseMatrix(Matrix matrix) {
    return Matrix(matrix.aa, matrix.ba, matrix.ca, matrix.ab, matrix.bb, matrix.cb, matrix.ac, matrix.bc, matrix.cc);
}

void RigidBody::ComputeForceAndTorque(double t, RigidBody *rb) {
    //store torque in rb->torque
    //store sum of forces in rb->force
    //TODO: function that checks where magnets are and compute force that they have on object
    //Total force F(t) = Sum of all forces Fi(t)
    //Total external torque = Sum of( (ri(t) - x(t)) x Fi(t) ) -> volume times total force
    rb->force = sf::Vector3f(0.0f, 9.81f, 0.0f);
    rb->torque = calcCrossProd(rb->x, rb->force);
    // FindAllCollisions(contacts, ncontacts);
    // computeContactForces(contacts, ncontacts, t);
}

Quaternion RigidBody::normalizeQuant(Quaternion quaternion) {
    double length = calcMagnitude(sf::Vector3f(quaternion.i, quaternion.j, quaternion.k));
    sf::Vector3f vec = calcDivScalar(sf::Vector3f(quaternion.i, quaternion.j, quaternion.k), length);
    return Quaternion(quaternion.r, vec.x, vec.y, vec.z);
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
    return Matrix(aa, ab, ac, ba, bb, bc, ca, cb, cc);
}

Quaternion RigidBody::matrixToQuaternion(Matrix matrix) {
    Quaternion q;
    double tr, s;

    tr = matrix.aa + matrix.bb + matrix.cc;

    if(tr >= 0) {
        s = std::sqrt(tr + 1);
        q.r = 0.5 * s;
        s = 0.5 / s;
        q.i = (matrix.cb - matrix.bc) * s;
        q.j = (matrix.ac - matrix.ca) * s;
        q.k = (matrix.ba - matrix.ab) * s;
    }
    else {
        int i = 0;

        if(matrix.bb > matrix.aa) {
            i = 1;
        }

        if(i == 0) {
            if(matrix.cc > matrix.aa) {
                i = 2;
            }
        } else if(i == 1) {
            if(matrix.cc > matrix.bb) {
                i = 2;
            }
        }

        switch (i) {
            case 0:
                s = std::sqrt((matrix.aa - (matrix.bb + matrix.cc)) + 1);
                q.i = 0.5 * s;
                s = 0.5 / s;
                q.j = (matrix.ab + matrix.ba) * s;
                q.k = (matrix.ca + matrix.ac) * s;
                q.r = (matrix.cb - matrix.bc) * s;
                break;
            case 1:
                s = std::sqrt((matrix.bb - (matrix.cc + matrix.aa)) + 1);
                q.j = 0.5 * s;
                s = 0.5 / s;
                q.k = (matrix.bc + matrix.cb) * s;
                q.i = (matrix.ab + matrix.ba) * s;
                q.r = (matrix.ac - matrix.ca) * s;
                break;
            case 2:
                s = std::sqrt((matrix.cc - (matrix.aa + matrix.bb)) + 1);
                q.k = 0.5 * s;
                s = 0.5 / s;
                q.i = (matrix.ca + matrix.ac) * s;
                q.j = (matrix.bc + matrix.cb) * s;
                q.r = (matrix.ba - matrix.ab) * s;
                break;
        }
    }

    return q;
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
    return Quaternion(q.r * scalar, q.i * scalar, q.j * scalar, q.k * scalar);
}

Matrix RigidBody::calcIbody() {
    if(this->type == 1) //is rectangular block
    {
        std::cout << "calcIbody - its a rectangle" << std::endl;
        //take inetia tensor formular for blocks - since its only 2D - z = 0
        return Matrix((this->height * this->height), 0.0f, 0.0f, 0.0f, (this->width * this->width), 0.0f, 0.0f, 0.0f,
                              ((this->width * this->width)+(this->height * this->height)) * (this->mass / 12));
    }
    else {
        //TODO: do this for circle take inetia tensor formular for blocks - since its only 2D - z = 0
        return Matrix((this->height * this->height) * (this->mass / 12), 0.0f, 0.0f, 0.0f,
                      (this->width * this->width) * (this->mass / 12), 0.0f, 0.0f, 0.0f,
                      ((this->width * this->width)+(this->height * this->height)) * (this->mass / 12));
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
        std::cout << "Determinante is zero!" << std::endl;
        return Matrix(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
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
            return Matrix(adj.aa * (1/det), adj.ab * (1/det), adj.ac * (1/det), adj.ba * (1/det), adj.bb * (1/det),
                          adj.bc * (1/det), adj.ca * (1/det), adj.cb * (1/det), adj.cc * (1/det));
        }
    }
}

extern sf::RenderWindow* mainWindow;
float t = 0;
void RigidBody::RunSimulation(float deltaTime)
{
    std::cout << "Run simulation" << std::endl;
    double x0[STATE_SIZE * NBODIES], xFinal[STATE_SIZE * NBODIES];
    std::cout << "Size of rigid_bodies: " << rigid_bodies->size() << std::endl;
    BodiesToArray(xFinal);

    t += deltaTime;
    for(int i = 0; i < STATE_SIZE * NBODIES; i++) {
        x0[i] = xFinal[i];
    }

    ode(x0, xFinal, STATE_SIZE * NBODIES, t-deltaTime, t);
    ArrayToBodies(xFinal);
}

void RigidBody::DisplayBodies(sf::RenderWindow &window) {
    std::cout << "Display Bodies" << std::endl;
    //loop through all bodies and display them
    //player.Draw(app);
    for (int i = 0; i < rigid_bodies->size(); i++) {
        sf::RectangleShape body;
        body.setFillColor(sf::Color(100.0f, 0.0f, 0.0f));
        body.setSize(sf::Vector2f(70.0f, 70.0f));
        std::cout << "Position of rectangle  X: " << rigid_bodies->at(i)->getPosition().x << " Y: " << rigid_bodies->at(i)->getPosition().y
        << " Z: " << rigid_bodies->at(i)->getPosition().z << std::endl;
        body.setPosition(rigid_bodies->at(i)->getPosition().x + body.getPosition().x,
                         rigid_bodies->at(i)->getPosition().y + body.getPosition().y);
        rigid_bodies->at(i)->resetPosition();
        window.draw(body);
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
    double h = t1 - t0;
    double x_half[len], v_half[len];

    // Compute initial velocity
    double v0[len], k1[len];
    rigid_bodies->at(0)->Dxdt(t0, x0, k1);
    for (int i = 0; i < len; i++) {
        v0[i] = x0[i] + 0.5*h*k1[i];
    }

    // Update position and velocity using Verlet integration
    for (int i = 0; i < len; i++) {
        x_half[i] = x0[i] + h*v0[i];
    }
    rigid_bodies->at(0)->Dxdt(t1, x_half, k1);
    for (int i = 0; i < len; i++) {
        v_half[i] = v0[i] + 0.5*h*k1[i];
    }
    for (int i = 0; i < len; i++) {
        xEnd[i] = x_half[i] + h*v_half[i];
    }
}

void RigidBody::resetPosition() {
    this->x = sf::Vector3f(0.0f, 0.0f, 0.0f);
}

//TODO: can i make + like that?
sf::Vector3f RigidBody::pt_velocity(RigidBody *body, sf::Vector3f p) {
    return body->v + calcCrossProd(body->omega, sf::Vector3f(p.x - body->x.x, p.y - body->x.y, p.z - body->x.z));
}

// bool RigidBody::colliding(Contact *c, sf::Vector3f p) {
//     sf::Vector3f padot = pt_velocity(c->a, p);
//     sf::Vector3f pbdot = pt_velocity(c->b, p);
//     double vrel = calcDotProd(c->n, (padot - pbdot)); //TODO: can i write - like that?

//     if(vrel > 1) //moving away
//         return false;
//     if(vrel > -1) //resting contact
//         return false;
//     else
//         return true;
// }

// void RigidBody::collision(Contact *c, double epsilon, sf::Vector3f p) {
//     sf::Vector3f padot = pt_velocity(c->a, c->p);
//     sf::Vector3f pbdot = pt_velocity(c->b, c->p);
//     sf::Vector3f n = c->n;
//     sf::Vector3f ra = p - c->a->x;
//     sf::Vector3f rb = p - c->b->x;
//     double vrel = calcDotProd(n, padot - pbdot);
//     double numerator = -(1 + epsilon) * vrel;

//     //Calculate denominator in four parts
//     double term1 = 1 / c->a->mass;
//     double term2 = 1 / c->b->mass;
//     double term3 = n * (calcCrossProd(c->a->Iinv * calcCrossProd(ra, n)), ra);
//     double term4 = n * (calcCrossProd(c->b->Iinv * calcCrossProd(rb, n)), rb);

//     //Compute the impulse magnitude
//     double j = numerator / (term1 + term2 + term3 + term4);
//     sf::Vector3f force = sf::Vector3f(n.x * j, n.y * j, n.z * j);

//     //Apply the impulse to the bodies
//     c->a->P += force;
//     c->b->P -= force;
//     c->a->L += calcCrossProd(ra, force);
//     c->b->L -= calcCrossProd(rb, force);

//     //recompute auxiliary variables
//     c->a->v = sf::Vector3f(c->a->P.x / c->a->mass, c->a->P.y / c->a->mass, c->a->P.z / c->a->mass);
//     c->b->v = sf::Vector3f(c->b->P.x / c->b->mass, c->b->P.y / c->b->mass, c->b->P.z / c->b->mass);
//     c->a->omega = calcMatrixMult(c->a->Iinv, c->a->L);
//     c->b->omega = calcMatrixMult(c->b->Iinv, c->b->L);
// }

// /*void RigidBody::FindAllCollisions(Contact *contacts, int ncontacts) {
//     bool had_collision;
//     double epsilon = 0.5;

//     do {
//         had_collision = false;

//         for(int i = 0; i < ncontacts; i++) {
//             if(colliding(&contacts[i])) {
//                 collision(&contacts[i], epsilon, sf::Vector3f(0,0,0)); //TODO: p parameter
//                 had_collision = true;

//                 //Tell the solver we had a collision
//                 ode_discontinues(); //TODO: what is that?
//             }
//         }
//     } while(had_collision == true);
// }*/

// void RigidBody::computeContactForces(Contact *contacts, int ncontacts, double t) {
//     //we assume that every element of contacts[] represents a contact in resting contact
//     //Also we assume that for each element in rigid_bodies[] the force and torque fileds
//     //have been set to the net external force and torque acting on the body due to gravity,
//     //wind etc. perhaps by a call to
//     //computeExternalForceAndTorqueForAllBodies(t); //TODO: implement this method?

//     //allocate nxn matrix amat and n-vectors fvec and bvec

//     Matrix *amat = new Matrix(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
//     std::vector<double> *bvec = new std::vector<double>(ncontacts); //TODO: is this really filled with doubles?
//     std::vector<double> *fvec = new std::vector<double>(ncontacts); //TODO: is this really filled with doubles?

//     //compute aij and bi coefficients
//     compute_a_matrix(contacts, ncontacts, *amat);
//     compute_b_vector(contacts, ncontacts, *bvec);

//     //solve for fj's
//     qp_solve(*amat, *bvec, *fvec);

//     //Now add the resting contact forces we jsut computed into the force and torque field of each body
//     for(int i = 0; i < ncontacts; i++) {
//         double f = fvec->at(i); //fi
//         sf::Vector3f n = contacts[i].n;
//         RigidBody *A = contacts[i].a;
//         RigidBody *B = contacts[i].b;

//         //apply the force f n positively to A
//         A->force += sf::Vector3f(n.x * f, n.y * f, n.z * f);
//         A->torque += sf::Vector3f((contacts[i].p - A->x).x * sf::Vector3f(n.x * f, n.y * f, n.z * f).x,
//                                   (contacts[i].p - A->x).y * sf::Vector3f(n.x * f, n.y * f, n.z * f).y,
//                                   (contacts[i].p - A->x).z * sf::Vector3f(n.x * f, n.y * f, n.z * f).z);

//         //and negatively to B
//         B->force -= sf::Vector3f(n.x * f, n.y * f, n.z * f);
//         B->torque -=  sf::Vector3f((contacts[i].p - B->x).x * sf::Vector3f(n.x * f, n.y * f, n.z * f).x,
//                                    (contacts[i].p - B->x).y * sf::Vector3f(n.x * f, n.y * f, n.z * f).y,
//                                    (contacts[i].p - B->x).z * sf::Vector3f(n.x * f, n.y * f, n.z * f).z);
//     }
// }

// /*sf::Vector3f RigidBody::computeNdot(Contact *c) {
//     if(c->vf) //vertex/face contact
//     {
//         //the vector n is attached to B so..
//         return calcCrossProd(c->b->omega, c->n);
//     }
//     else {
//         sf::Vector3f eadot = calcCrossProd(c->a->omega, ea); //TODO: define ea
//         sf::Vector3f ebdot = calcCrossProd(c->b->omega, eb);
//         sf::Vector3f n1 = ea * eb;
//         sf::Vector3f z = (eadot * eb) + (ea * ebdot);
//         double l = calcMagnitude(n1);
//         n1 = calcDivScalar(n1, l);

//         return (z - ((z * n) * n)) / l; //TODO define n?
//     }
// }*/

// void RigidBody::compute_b_vector(Contact *contacts, int ncontacts, std::vector<double> &b) {
//     for(int i = 0; i < ncontacts; i++) {
//         Contact *c = &contacts[i];
//         RigidBody *A = c->a;
//         RigidBody *B = c->b;
//         sf::Vector3f n = c->n;
//         sf::Vector3f ra = c->p - A->x;
//         sf::Vector3f rb = c->p - B->x;

//         //Get the external forces and torques
//         sf::Vector3f f_ext_a = A->force;
//         sf::Vector3f f_ext_b = B->force;
//         sf::Vector3f t_ext_a = A->torque;
//         sf::Vector3f t_ext_b = B->torque;

//         sf::Vector3f a_ext_part, a_vel_part, b_ext_part, b_vel_part;

//         //compute the part of p..a(t0) due to the external force and torque
//         a_ext_part = sf::Vector3f(f_ext_a.x / A->mass, f_ext_a.y / A->mass, f_ext_a.z / A->mass)
//                 + calcCrossProd(calcMatrixMult(A->Iinv, t_ext_a), ra);
//         b_ext_part = sf::Vector3f(f_ext_b.x / B->mass, f_ext_b.y / B->mass, f_ext_b.z / B->mass)
//                      + calcCrossProd(calcMatrixMult(B->Iinv, t_ext_b), rb);

//         //Compute the part of p..a(to) due to velocity
//         a_vel_part = calcCrossProd(A->omega, (calcCrossProd(A->omega, ra))) +
//                 calcCrossProd((calcMatrixMult(A->Iinv, (calcCrossProd(A->L, A->omega)))), ra);
//         b_vel_part = calcCrossProd(B->omega, (calcCrossProd(B->omega, rb))) +
//                      calcCrossProd((calcMatrixMult(B->Iinv, (calcCrossProd(B->L, B->omega)))), rb);

//         //combine the above results and dot with nî(t0)
//         double k1 = calcDotProd(((a_ext_part + a_vel_part) - (b_ext_part + b_vel_part)), n);
//         sf::Vector3f ndot = computeNdot(c);
//         double k2 = calcDotProd(sf::Vector3f(ndot.x * 2, ndot.y * 2, ndot.z * 2),
//                                 ((pt_velocity(A, c->p) - pt_velocity(B, c->p))));

//         b[i] = k1 + k2;

//     }
// }

// /*void RigidBody::compute_a_matrix(Contact *contacts, int ncontacts, Matrix &a) {
//     for(int i = 0; i < ncontacts; i++) {
//         for(int j = 0; j < ncontacts; j++) {
//             //TODO: this is not correct:
//             a[i,j] = compute_aij(contacts[i], contacts[j]);
//         }
//     }
// }*/

// /*double RigidBody::compute_aij(Contact ci, Contact cj) {
//     //if the bodies involved in the ith and jth contact are distinct, then aij is zero
//     if((ci.a != cj.a) && (ci.b != cj.b) && (ci.a != cj.b) && (ci.b != cj.a)) {
//         return 0.0;
//     }

//     RigidBody *A = ci.a;
//     RigidBody *B = ci.b;
//     sf::Vector3f ni = ci.n;
//     sf::Vector3f nj = cj.n;
//     sf::Vector3f pi = ci.p;
//     sf::Vector3f pj = cj.p;
//     sf::Vector3f ra = pi - A->x;
//     sf::Vector3f rb = pi - B->x;

//     sf::Vector3f force_on_a = sf::Vector3f(0.0f, 0.0f, 0.0f);
//     sf::Vector3f torque_on_a = sf::Vector3f(0.0f, 0.0f, 0.0f);

//     if(cj.a == ci.a) {
//         //force direction of jth contact force on A
//         force_on_a = nj;
//         //torque direction
//         torque_on_a = calcCrossProd(pj - A->x, nj);
//     } else if(cj.b == ci.a) {
//         force_on_a = sf::Vector3f(nj.x * -1, nj.y * -1, nj.z * -1);
//         torque_on_a = calcCrossProd(pj - A->x, nj);
//     }

//     sf::Vector3f force_on_b = sf::Vector3f(0.0f, 0.0f, 0.0f);
//     sf::Vector3f torque_on_b = sf::Vector3f(0.0f, 0.0f, 0.0f);

//     if(cj.a == ci.b) {
//         //force direction of jth contact force on A
//         force_on_b = nj;
//         //torque direction
//         torque_on_b = calcCrossProd(pj - B->x, nj);
//     } else if(cj.b == ci.b) {
//         force_on_b = sf::Vector3f(nj.x * -1, nj.y * -1, nj.z * -1);
//         torque_on_b = calcCrossProd(pj - B->x, nj);
//     }

//     //Now compute how the jth contact force affects the linear and angular acceleration of the
//     //contact point on body A
//     sf::Vector3f a_linear = sf::Vector3f(force_on_a.x / A->mass, force_on_a.y / A->mass, force_on_a.z / A->mass);
//     //Same for B
//     sf::Vector3f b_linear = sf::Vector3f(force_on_b.x / B->mass, force_on_b.y / B->mass, force_on_b.z / B->mass);

//     return ni * ((a_linear + a_angular) - (b_linear + b_angular)); //TODO: where is a_angular and b_angular??
// }*/