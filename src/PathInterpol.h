#include <SFML/Graphics.hpp>
#include "assert.h"

class SplineSegment
{
    public:
        float total_length_ = 0;
        std::vector<float> arc_lengths_;

        SplineSegment(sf::Vector2f p0, sf::Vector2f p1, sf::Vector2f p2, sf::Vector2f p3)
        {
            ctrl_points_[0] = p0;
            ctrl_points_[1] = p1;
            ctrl_points_[2] = p2;
            ctrl_points_[3] = p3;

            initArcLengths();
        }

        sf::Vector2f ctrl_points_[4];

        std::vector<sf::Vector2f> arc_points_;

        sf::Vector2f getPoint(float t);
        void drawControlPoints();
        void drawCurve();
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
        int current_spline_ = 0;
        float time_;
        sf::Vector2f current_pos_;

        float total_length_ = 0;
        std::vector<ArcLengthTableEntry> arc_length_table_;

        Spline(std::vector<sf::Vector2f> ctrl_points)
        {
            this->ctrl_points_ = ctrl_points;
            initSegments();
            initArcLengthTable();
            printTable();
        }

        void initSegments();

        void interpolate(float time_delta);

        void drawObject();
        void drawControlPoints();
        void drawCurve();

        void printLengths();

        void initArcLengthTable();
        std::pair<int, float> searchForU(float arc_length);
        void printTable();
};