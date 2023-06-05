#include "PathInterpol.h"
// #include "RigidBody.h"
#include "ParticleDynamics.h"
#include "objects/Magnet.h"
#include "SFML/Graphics.hpp"
#include <vector>

#ifndef MAGNITY_LEVEL_H
#define MAGNITY_LEVEL_H

class Level {
    public:
        std::vector<Spline*> splines_;
        ParticleDynamics* particle_dynamics_;
        // std::vector<RigidBody*> rigid_bodies_;

        std::vector<sf::Texture*> loaded_textures_;
        
        ForceSource* mouse_force = nullptr;
        // magnets

        Magnet* magnet1 = nullptr;
        Magnet* magnet2 = nullptr;


        Level();

        void destroy();

        void update(float time_delta);
        
        void updateMouseParticlePosition(sf::Vector2f new_pos);

        void draw(sf::RenderWindow& renderWindow, float delta_time);

        static Level* LoadLevel1(sf::View& view);
};


#endif // MAGNITY_LEVEL_H