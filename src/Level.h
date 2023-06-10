#include "TGUI/TGUI.hpp"
#include "SFML/Graphics.hpp"
#include <SFML/Audio.hpp>

#include "PathInterpol.h"
#include "RigidBody.h"
#include "ParticleDynamics.h"
#include "objects/Magnet.h"
#include "gameplay/MagnetArea.h"
#include "gameplay/WallArea.h"
#include <vector>

#ifndef MAGNITY_LEVEL_H
#define MAGNITY_LEVEL_H

class Level {
    public:
        std::vector<Spline*> splines_;
        std::vector<ParticleDynamics*> particle_dynamics_;
        std::vector<RigidBody*> rigid_bodies_;

        std::vector<sf::Texture*> loaded_textures_;
        std::vector<Magnet*> magnets_;
        MagnetArea* magnet_area_ = nullptr;
        WallArea* wall_area_ = nullptr;
        
        std::vector<SpriteObject*> game_objects_;

        GameObject* object_ = nullptr;
        SpriteObject* target_area_ = nullptr;

        ForceSource* mouse_force = nullptr;

        sf::Color background_color_ = sf::Color::White;
        tgui::Panel::Ptr level_panel_;

        std::string name;

        sf::Music background_music_;
        sf::Music level_complete_sound_;
        sf::Music level_failed_sound_;

        Level(std::string name);

        void destroy(sf::RenderWindow& window);

        void update(float time_delta);
        
        void handlePolledKeyInput(sf::Event keyEvent);
        void handleInstantKeyInput(float delta_time);
        void handleClick(sf::Vector2f mouse_position);
        void handleRelease();
        void handleDrag(sf::Vector2f mouse_position);

        void updateMouseParticlePosition(sf::Vector2f new_pos);

        void draw(sf::RenderWindow& window, float delta_time);

        void loadFromFile(std::string file_name);

        static Level* LoadLevel(sf::RenderWindow& window, tgui::GuiSFML& gui, Level* (*levelToLoad)(sf::RenderWindow& window, tgui::GuiSFML& gui));
        static Level* LoadLevel0(sf::RenderWindow& window, tgui::GuiSFML& gui);
        static Level* LoadLevel1(sf::RenderWindow& window, tgui::GuiSFML& gui);
        static Level* LoadLevel2(sf::RenderWindow& window, tgui::GuiSFML& gui);
        static Level* LoadLevel3(sf::RenderWindow& window, tgui::GuiSFML& gui);
        static Level* LoadLevel4(sf::RenderWindow& window, tgui::GuiSFML& gui);
        static Level* LoadLevelParticleDemo(sf::RenderWindow& window, tgui::GuiSFML& gui);
        static Level* LoadLevelPathInterpolDemo(sf::RenderWindow& window, tgui::GuiSFML& gui);
        static Level* LoadLevelRigidBodyDemo(sf::RenderWindow& window, tgui::GuiSFML& gui);
        static Level* LoadLevelVoronoiDemo(sf::RenderWindow& window, tgui::GuiSFML& gui);
};


#endif // MAGNITY_LEVEL_H