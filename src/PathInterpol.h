#include <SFML/Graphics.hpp>
#include "assert.h"

extern sf::RenderWindow* mainWindow;

class Spline
{
    public:
        Spline(sf::Vector2f p0, sf::Vector2f p1, sf::Vector2f p2, sf::Vector2f p3)
        {
            ctrl_points_[0] = p0;
            ctrl_points_[1] = p1;
            ctrl_points_[2] = p2;
            ctrl_points_[3] = p3;
        }

        sf::Vector2f ctrl_points_[4];

        sf::Vector2f getPoint(float t);
        void drawControlPoints();
        void drawCurve();
};

class Path
{
    public:
        std::vector<sf::Vector2f> ctrl_points_;
        std::vector<Spline> splines_;
        float traversal_speed_ = 2.f;
        int current_spline_ = 0;
        float time_;
        sf::Vector2f current_pos_;

        Path(std::vector<sf::Vector2f> ctrl_points)
        {
            this->ctrl_points_ = ctrl_points;
            initSplines();
        }

        void initSplines();

        void interpolate(float time_delta);

        void drawObject();
        void drawControlPoints();
        void drawCurve();
};