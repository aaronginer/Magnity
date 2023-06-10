#include <SFML/Graphics.hpp>
#include "assert.h"
#include <mutex>

class SplineSegment
{
    public:
        float total_length_ = 0;
        std::vector<float> arc_lengths_;

        sf::Vector2f ctrl_points_[4];

        std::vector<sf::Vector2f> arc_points_;

        SplineSegment(sf::Vector2f p0, sf::Vector2f p1, sf::Vector2f p2, sf::Vector2f p3);

        sf::Vector2f getPoint(float t);
        void drawSamples();
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

        float traversal_speed_ = 4.f;
        float time_;
        sf::Vector2f current_pos_;

        float total_length_ = 0;
        std::vector<ArcLengthTableEntry> arc_length_table_;

        sf::Texture ctrl_texture_;
        sf::Texture vector_texture_;
        std::vector<sf::Sprite> ctrl_sprites_;

        std::mutex mutex_;

        bool draw_curve_ = false;
        bool draw_ctrl_and_arc_ = false;

        int easing_option_ = 0;

        Spline(std::vector<sf::Vector2f> ctrl_points);

        void init();
        void initSegments();
        void initArcLengthTable();

        void interpolate(float time_delta);

        void drawObject();
        void drawControlPoints();
        void drawArcSamples();
        void drawCurve();

        void printLengths();

        std::pair<int, float> searchForU(float arc_length);
        void printTable();
};