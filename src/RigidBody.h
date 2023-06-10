///////////////////////////////////////////////////////////////////////////////////////
//  1d. Rigid-Body Dynamics                                                         //
//  Mathematical Concepts were taken from the lecture slides, as well as            //
//  the script "Physically Based Modeling by David Baraff from TeachCenter          //
///////////////////////////////////////////////////////////////////////////////////////

#include <SFML/Graphics.hpp>
#include <cmath>
#include <unistd.h>
#include "Matrix.h"
#include <vector>
#include "math.h"
#include "objects/GameObject.h"
#ifndef MAGNITY_RIGIDBODY_H
#define MAGNITY_RIGIDBODY_H

#define STATE_SIZE 13 //cos we use quarternions
//TODO: Body coordinates to world coordinates

class RigidBody {

    public:
        static std::vector<RigidBody*>* rigid_bodies;
        static bool draw_vectors_;

        RigidBody(double mass, double density, unsigned int type, double width, double height, const sf::Texture& texture,
                    bool fixed, double posX, double posY, int id);

        static double calcMagnitude(sf::Vector3<double> vec);
        static sf::Vector3<double> calcCrossProd(sf::Vector3<double> vec1, sf::Vector3<double> vec2);
        static void ComputeForceAndTorque(RigidBody *rb, std::vector<RigidBody*> *rigid_bodies);
        Matrix calcIbody() const;
        static void DisplayBodies(sf::RenderWindow &window, std::vector<RigidBody*> *rigid_bodies);
        static void ode(std::vector<RigidBody*> *y0, std::vector<RigidBody> *yEnd,  double t0,
                                           double t1, std::vector<RigidBody*>* rigid_bodies,
                                           std::vector<RigidBody*> *insertedBodies);
        sf::Vector3<double> normalizeVector(sf::Vector3<double> vec);
        void checkForCollisions(std::vector<RigidBody*>* rigid_bodies, std::vector<RigidBody*> *insertedBodies);
        static void applyVelocityVerletIntegration(RigidBody* rigid_body0, RigidBody* rigid_body1, double timestep, std::vector<RigidBody*> *rigid_bodies);
        void applyCollision(RigidBody* rigidBody1, RigidBody* rigidBody2, sf::Vector3<double> collision_point);
        static void updateRigidBodies(std::vector<RigidBody*> *rigidbodies, float total_time, float delta_time);
        static void drawVelocityArrows(sf::RenderWindow& window, std::vector<RigidBody*> *rigid_bodies);
        void drawArrow(sf::RenderWindow& window, sf::Texture& tex, sf::Vector2f position, sf::Vector2f P);
        void drawAngularMomentum(sf::RenderWindow& window, sf::Vector2f position, double P);

        //state variables
        bool fixed = false;
        double width = 100;
        double height = 100;
        double mass = 10; //mass of object
        sf::Vector3<double> linear_acceleration = {0, 0, 0}; //linear acceleration of object
        double angular_acceleration = 0.f; //angular acceleration of object
        sf::Vector3<double> x = {0, 0, 0}; //Position x(t)
        sf::Vector3<double> P = {0, 0, 0}; //P(t) linear momentum
        sf::Vector3<double> v = {0, 0, 0};; //v(t) linear velocity
        double L; //L(t) angular momentum
        double w; //w(t) angular velocity
        sf::Vector3<double> force = {0, 0, 0};; //Sum of forces on object
        sf::Vector3<double> torque_vec = {0, 0, 0};; //Torque - spin
        std::vector<std::pair<sf::Vector3<double>, sf::Vector3<double>>> force_points;
        double Inertia = 0.f;
        sf::RectangleShape body;
        unsigned int type = 0;
        double radius = 0;
        bool collision_found = false;
        bool splitter = false;
        int id = 0;
        sf::Image img;
        sf::Texture texture;
        std::string nameImg;
        bool visible = true;
        bool disabled = false;
        bool is_game_object = false;
        bool contact_border = false;
        sf::Sprite momentum_vector;
        bool force_already_applied = false;
        bool draw_vA = false;
        GameObject obj;

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
        //the center of mass. I depend on the orientation of the body but not on translation
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