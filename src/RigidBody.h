///////////////////////////////////////////////////////////////////////////////////////
//  1d. Rigid-Body Dynamics                                                         //
//  Game objects have linear and angular momentum                                   //
//  Our game detects collisions between objects using spherical or cuboid bounding proxies //
//  Our objects react to elastic collisions by changing their momentums based on    //
//  physical forces (bouncing off, adjusting rotation, etc.) using Velocity-Verlet  //
//  Integration                                                                     //
//  Required Visualizations (switchable): Current momentum vectors
///////////////////////////////////////////////////////////////////////////////////////

#include <SFML/Graphics.hpp>
#include <cmath>
#include "Quaternion.h"
#include "Matrix.h"
#include "Contact.h"
#ifndef MAGNITY_RIGIDBODY_H
#define MAGNITY_RIGIDBODY_H

#define STATE_SIZE 13 //cos we use quarternions
#define NBODIES 1

//TODO: Body coordinates to world coordinates

class RigidBody {

    public:
        RigidBody(float mass, float density, unsigned int type, float width, float height);
        void addToRigidBodies(RigidBody *rigidBody);
        void calcCenterOfMass();
        double calcMagnitude(sf::Vector3f vec); //length of vector
        sf::Vector3f calcUnitVector(sf::Vector3f vec);
        double calcDotProd(sf::Vector3f vec1, sf::Vector3f vec2);
        sf::Vector3f calcDivScalar(sf::Vector3f vec, double scalar);
        sf::Vector3f calcCrossProd(sf::Vector3f vec1, sf::Vector3f vec2);
        Matrix calcMatrixMult(Matrix matrix1, Matrix matrix2);
        sf::Vector3f calcMatrixMult(Matrix matrix1, sf::Vector3f vec);
        sf::Vector3f addVectors(sf::Vector3f vec1, sf::Vector3f vec2);
        sf::Vector3f multScalar(sf::Vector3f vec, double scalar);
        sf::Vector3f negateVector(sf::Vector3f vec); // lets vector point in opposite direction
        sf::Vector3f getPosition();
        Matrix calcTransponseMatrix(Matrix matrix);
        void StateToArray(RigidBody *rb, double *y);
        void ArrayToState(RigidBody *rb, double *y);
        void ArrayToBodies(double x[]);
        void BodiesToArray(double x[]);
        void Dxdt(double t, double x[], double xdot[]);
        void DdtStateToArray(RigidBody *rb, double *xdot);
        Matrix Star(sf::Vector3f vec);
        Quaternion matrixToQuaternion(Matrix matrix);
        //TODO:
        void ComputeForceAndTorque(double t, RigidBody *rb);
        //compute rb->force and rb->torque
        Quaternion normalizeQuant(Quaternion quaternion);
        Matrix QuaternionToMatrix(Quaternion quaternion);
        Quaternion calcQuatMult(Quaternion q1, Quaternion q2);
        Quaternion calcQuatMult(Quaternion q, double scalar);
        Matrix calcIbody();
        Matrix calcIinversebody();
        void RunSimulation(float deltaTime);
        void DisplayBodies(sf::RenderWindow &window);
        void ode(double x0[], double xEnd[], int len, double t0, double t1);
        void resetPosition();
        sf::Vector3f pt_velocity(RigidBody *body, sf::Vector3f p); //return velocity of point on rigid body
        bool colliding(Contact *c, sf::Vector3f p); //return true if bodies are colliding - set THRESHOLD
        void collision(Contact *c, double epsilon, sf::Vector3f p); //loop through all contact points
        void FindAllCollisions(Contact contacts[], int ncontacts);
        void compute_a_matrix(Contact contacts[], int ncontacts, Matrix &a);
        void compute_b_vector(Contact contacts[], int ncontacts, std::vector<double> &b);
        void qp_solve(Matrix &a, std::vector<double> &b, std::vector<double> &f);
        void computeContactForces(Contact contacts[], int ncontacts, double t);
        sf::Vector3f computeNdot(Contact *c); //return derivative of the normal vector
        double compute_aij(Contact ci, Contact cj);


        double calcInertia(sf::Vector3f point_force, sf::Vector3f force);
        double calcTau(sf::Vector3f point_force, sf::Vector3f force);
        unsigned int getID();
        unsigned int getType();

    private:
        unsigned int id;
        unsigned int type;
        float density;
        float width;
        float height;
        float radius;


        ////RIGID BODY SHEET - 3D ----------------------------------------------------------------------
        //constant quantities
        double mass;
        Matrix Ibody;
        Matrix Ibodyinv;
        //state variables
        sf::Vector3f x; //center of mass x(t)
        Quaternion q;//use quaterinion instead of std::vector<sf::Vector3f> R; //R(t)
        Matrix R; //old Rotation matrix - needed for some calculations
        sf::Vector3f P; //P(t)
        sf::Vector3f L; //L(t)
        //Derived quantities (auxiliary variables)
        Matrix Iinv; //I^-1(t) inertia
        sf::Vector3f v; //v(t)
        sf::Vector3f omega; //w(t)
        //Computed quantities
        sf::Vector3f force;
        sf::Vector3f torque;

        ////spatial variables
        //Center of mass at (0,0,0)
        //Rotation matrix R(t) = 3x3 matrix
        //center of mass in body space = (0,0,0) in world space = ?
        //R(t)r -> r is fixed vector in body space
        //a body space point p0 is in world space p(t) = R(t) * p0 + x(t)
        //Center of mass in world space = x(t)
        //R(t) in body space can be (1,0,0) für x-axis, (0,1,0) für y-axis, (0,0,1) for z-axis is FIXED
        //R(t) in world space = (rxx, rxy, rxz) for x-axis, (ryx, ryy, ryz) for y-axis, (rzx, rzy, rzz) for z-axis
        //If you want to get the axis in real world compute for x' = R(t) * x usw.

        ////linear velocity
        //we now need to calculate x.(t) = velocity of center of mass in world space = v(t) = linear velocity
        //in body space, the orientation (=rotation) is FIXED, therefor body can in body space only be translated
        // v(t) = d/dt x(t)

        ////angular velocity
        //If there is rotation/spin, position of center of mass stays the same because we spin about
        //an axis that passes through center of mass
        //spin = w(t)
        //direction of w(t) = direction of axis, about which body is spinning
        //length/magnitude of w(t) = how fast body is spinning in revolution/time
        //we need a R.(t) = w(t) * R(t)

        ////mass of body
        //imagine body consists of a large number of particles 1 to N
        //particle has body space coordinate r0i and thats in world space
        //ri(t) = R(t) * r0i + x(t)
        //Total mass of Body = M = Sum of mi -> sum of all masses of particles

        ////Velocity of a Particle
        //we now need the velocity of a particle ri
        // = ri(t) = w(t) x (ri(t) - x(t)) + v(t) -> has linear and angular component

        ////Center of Mass
        //Center of Mass of a body in world space = (Sum(mi * ri(t)) / M
        //in Body space = (0,0,0)

        ////Force and Torque
        //Fi(t) = total external forces acting on ith particle at time t
        //Ti(t) = (ri(t) - x(t)) x Fi(t) = external torque acting on the ith particle
        //Torque is direction that represents axis the body would spin around due to force F if center of mass stays the same
        //Total force F(t) = Sum of all forces Fi(t)
        //Total external torque = Sum of( (ri(t) - x(t)) x Fi(t) )

        ////Linear Momentum
        //linear momentum p = m * v (mass * velocity) of one particle
        //Total linear momentum of body = P(t) = M * v(t)
        //Therefor P.(t) = F(t)

        ////Angular Momentum
        //Total angular momentum L(t) = I(t) * w(t)
        //I = inertia tensor = 3x3 matrix - describes how mass is distributed in body relative to
        //the center of mass. I depends on the orientation of the body but not on translation
        //L.(t) = T(t) -> therefor also P.(t)  = F(t)

        ////Inertia Tensor
        //is scaling factor between angular momentum and angular velocity
        //r'(t) is the displacement of the ith particle from center of mass x(t)
        //r'(t) = ri(t) - x(t)
        //I(t) = Sum of( mi * ( (ri'^T * ri') * 3x3 Einheitsmatrix - ri' * ri'^T )
        //Declare Ibody = Sum of( mi * ( (r0i^T * r0i) * 3x3 Einheitsmatrix - r0i * r0i^T ) -> this is in body space
        //Compute Ibody at beginning because this stays the same all the time
        //Then I(t) = R(t) * Ibody * R(t)^T

        ////Rigid Body Equations of Motion
        //Now we can define the state Vector X(t) =
        //
        //                  | x(t) center of mass   |
        //                  | R(t) rotation matrix  |
        //                  | P(t) linear momentum  |
        //                  | L(t) angular momentum |
        //
        // in each step calculate I(t), w(t) and v(t)
        // v(t) = P(t) / M      I(t) = R(t) * Ibody * R(t)^T        w(t) = I(t)^-1 * L(t)
        //
        // Derivative d/dt X(t) is:
        //
        //                  | x(t) center of mass   |       | v(t) linear velocity |
        //               d  | R(t) rotation matrix  |  =    | w(t) * R(t)          |
        //              dt  | P(t) linear momentum  |       | F(t) sum of forces   |
        //                  | L(t) angular momentum |       | T(t) torque          |
        //
        //// !Remark: Instead of using R(t) it is better to use quaternions

        ////Quaternions
        //s + vxi + vyj + yzk = [s, v]
        //[s1, v1] * [s2, v2] = [s1 * s2 - v1 • v2,  s1 * v2 + s2 * v1 + v1 x v2]
        //unit quaternion: [cos(theta / 2), sin/theta / 2) * u] -> u = unit axis where body rotates around
        //q1 * q2 means first rotation q1 followed by rotation q2
        //q.(t) = 1/2 * w(t) * q(t) -> (w(t) * q(t) = [0,w(t)] * [s,v])

};


#endif //MAGNITY_RIGIDBODY_H
