#include <SFML/Graphics.hpp>
#include "assert.h"
#include "objects/SpriteObject.h"
#include <mutex>

#ifndef MAGNITY_PATH_H
#define MAGNITY_PATH_H

class SplineSegment
{
    public:
        float total_length_ = 0;
        std::vector<float> arc_lengths_;

        sf::Vector2f ctrl_points_[4];

        std::vector<sf::Vector2f> arc_points_;

        SplineSegment(sf::Vector2f p0, sf::Vector2f p1, sf::Vector2f p2, sf::Vector2f p3);

        sf::Vector2f getPoint(float t);
        void drawSamples(sf::RenderWindow& window);
        void initArcLengths();
};

typedef struct
{
    int segment_index_;
    float segment_parameter_;
    float arc_length_;
} ArcLengthTableEntry;

class Spline
{
    public:
        std::vector<sf::Vector2f> ctrl_points_;
        std::vector<SplineSegment> segments_;
        float time_ = 0.0f;
        sf::Vector2f current_pos_;

        float total_length_ = 0;
        std::vector<ArcLengthTableEntry> arc_length_table_;

        sf::Texture ctrl_texture_;
        std::vector<sf::Sprite> ctrl_sprites_;
        SpriteObject* sprite_;

        std::mutex mutex_;

        static bool draw_curve_;
        static bool draw_ctrl_and_arc_;
        static float traversal_speed_;

        static int easing_option_;
    
        static sf::Vector2f* ctrl_point_to_drag;
        static sf::Sprite* ctrl_sprite_to_drag;
        static Spline* ctrl_spline_to_drag;

        Spline(std::vector<sf::Vector2f> ctrl_points, sf::Texture& texture, bool circular=false);

        ~Spline() { delete sprite_; }

        void init();
        void initSegments();
        void initArcLengthTable();

        void update(float time_delta);

        void drawObject(sf::RenderWindow& window);
        void drawControlPoints(sf::RenderWindow& window);
        void drawArcSamples(sf::RenderWindow& window);
        void drawCurve(sf::RenderWindow& window);

        void printLengths();

        std::pair<int, float> searchForU(float arc_length);
        void printTable();

        void handleClick(sf::Vector2f mouse_position);
        static void handleRelease();
        static void handleDrag(sf::Vector2f mouse_position);
};

#endif // MAGNITY_PATH_H