#include "PathInterpol.h"

sf::Vector2f Spline::getPoint(float t)
{
    return 0.5f * (
        (2.0f * ctrl_points_[1]) + 
        (-ctrl_points_[0] + ctrl_points_[2]) * t +
        (2.0f * ctrl_points_[0] - 5.0f * ctrl_points_[1] + 4.0f * ctrl_points_[2] - ctrl_points_[3]) * t * t +
        (-ctrl_points_[0] + 3.0f * ctrl_points_[1] - 3.0f * ctrl_points_[2] + ctrl_points_[3]) * t * t * t
    );
}

void Spline::drawControlPoints()
{
    for (size_t i = 0; i < 4; i++)
    {
        sf::RectangleShape rect;
        rect.setSize(sf::Vector2f(5.f, 5.f));
        rect.setPosition(ctrl_points_[i]);
        rect.setFillColor(sf::Color::Blue);
        (*mainWindow).draw(rect);
    }
}

void Spline::drawCurve()
{
    float t = 0;
    for (size_t i = 0; i < 100; i++)
    {
        sf::CircleShape circle(2.0f);
        circle.setFillColor(sf::Color::Green);
        circle.setPosition(getPoint(t));
        (*mainWindow).draw(circle);
        t+=1.f/100;
    }
}

void Path::initSplines()
{
    assert(ctrl_points_.size() >= 4 && "The path needs at least 4 control points.");

    for (size_t i = 0; i < ctrl_points_.size() - 3; i++)
    {
        splines_.push_back(Spline(ctrl_points_[i], ctrl_points_[i+1], ctrl_points_[i+2], ctrl_points_[i+3]));
    }
}

void Path::interpolate(float time_delta)
{
    time_+= time_delta * 1/traversal_speed_;
    if (time_ > 1.f)
    {
        current_spline_ = (current_spline_ + 1) % splines_.size();
        time_ -= 1.f;
    }

    current_pos_ = splines_[current_spline_].getPoint(time_);            
}

void Path::drawObject()
{
    sf::CircleShape circle(7.0f);
    circle.setFillColor(sf::Color::Red);
    circle.setPosition(current_pos_);

    (*mainWindow).draw(circle);
}

void Path::drawControlPoints()
{
    for (size_t i = 0; i < ctrl_points_.size(); i++)
    {
        sf::RectangleShape rect;
        rect.setSize(sf::Vector2f(7.f, 7.f));
        rect.setPosition(ctrl_points_.at(i));
        rect.setFillColor(sf::Color::Blue);
        (*mainWindow).draw(rect);
    }
}

void Path::drawCurve()
{
    for (size_t i = 0; i < splines_.size(); i++)
    {
        splines_.at(i).drawCurve();
    }
}