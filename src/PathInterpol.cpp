#include "PathInterpol.h"
#include "cmath"

extern sf::RenderWindow* mainWindow;

#define ARC_LENGTH_SAMPLES 30ULL

float distance(sf::Vector2f v1, sf::Vector2f v2)
{
    sf::Vector2f diff = v2-v1;
    return sqrt(pow(diff.x, 2) + pow(diff.y, 2));
}

sf::Vector2f SplineSegment::getPoint(float t)
{
    return 0.5f * (
        (2.0f * ctrl_points_[1]) + 
        (-ctrl_points_[0] + ctrl_points_[2]) * t +
        (2.0f * ctrl_points_[0] - 5.0f * ctrl_points_[1] + 4.0f * ctrl_points_[2] - ctrl_points_[3]) * t * t +
        (-ctrl_points_[0] + 3.0f * ctrl_points_[1] - 3.0f * ctrl_points_[2] + ctrl_points_[3]) * t * t * t
    );
}

#define CURVE_POINTS 300
void SplineSegment::drawCurve()
{
    for (size_t i = 0; i < CURVE_POINTS; i++)
    {
        sf::CircleShape circle(1.0f);
        circle.setFillColor(sf::Color::Green);
        circle.setPosition(getPoint((float)i/(CURVE_POINTS-1)));
        (*mainWindow).draw(circle);
    }
}

void SplineSegment::initArcLengths()
{
    sf::Vector2f prev_point = getPoint(0);

    for (size_t i = 0; i < ARC_LENGTH_SAMPLES; i++)
    {
        sf::Vector2f next_point = getPoint((float)i/(ARC_LENGTH_SAMPLES-1));
        arc_points_.push_back(next_point);
        printf("(%f %f)(%f %f), d: %f\n", prev_point.x, prev_point.y, next_point.x, next_point.y, distance(prev_point, next_point));
        total_length_ += distance(prev_point, next_point);
        arc_lengths_.push_back(total_length_);

        prev_point = next_point;
    }
}

void Spline::initSegments()
{
    assert(ctrl_points_.size() >= 4 && "The path needs at least 4 control points.");

    for (size_t i = 0; i < ctrl_points_.size() - 3; i++)
    {
        segments_.push_back(SplineSegment(ctrl_points_[i], ctrl_points_[i+1], ctrl_points_[i+2], ctrl_points_[i+3]));
    }
}

void Spline::interpolate(float time_delta)
{
    time_+= time_delta * 1/traversal_speed_;
    if (time_ > 1.f)
    {
        current_spline_ = (current_spline_ + 1) % segments_.size();
        time_ -= 1.f;
    }

    float t = -(cos(M_PI * time_) - 1) / 2; 
    t = time_ < 0.5 ? 4 * time_ * time_ * time_ : 1 - pow(-2 * time_ + 2, 3) / 2;

    std::pair<int, float> interpol = searchForU(total_length_ * t);
    current_pos_ = segments_[interpol.first].getPoint(interpol.second);          
}

void Spline::drawObject()
{
    sf::CircleShape circle(7.0f);
    circle.setFillColor(sf::Color::Red);
    circle.setPosition(current_pos_);

    (*mainWindow).draw(circle);
}

void Spline::drawControlPoints()
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

void Spline::drawCurve()
{
    for (size_t i = 0; i < segments_.size(); i++)
    {
        segments_.at(i).drawCurve();
    }
}

void Spline::printLengths()
{
    for (size_t i = 0; i < segments_.size(); i++)
    {
        printf("Length [%ld]: %f (%f,%f) (%f,%f)\n ", i, segments_[i].total_length_, segments_[i].ctrl_points_[1].x, segments_[i].ctrl_points_[1].y, segments_[i].ctrl_points_[2].x, segments_[i].ctrl_points_[2].y);
    }
}

void Spline::initArcLengthTable()
{
    size_t sample_index_ = 0;
    for (size_t i = 0; i < segments_.size(); i++)
    {
        std::vector<float>& arc_lengths = segments_[i].arc_lengths_;
        
        for (size_t j = 0; j < arc_lengths.size(); j++)
        {
            arc_length_table_.push_back({(int)i, (float) j / (ARC_LENGTH_SAMPLES-1), total_length_ + arc_lengths[j]});
            sample_index_++;
        }
        total_length_ += segments_[i].total_length_;
    }
}

std::pair<int, float> Spline::searchForU(float arc_length)
{
    size_t i = 1;
    for (; i < arc_length_table_.size(); i++)
    {
        if (arc_length >= arc_length_table_[i-1].arc_length_ && arc_length <= arc_length_table_[i].arc_length_ 
            && arc_length_table_[i-1].segment_index_ == arc_length_table_[i].segment_index_)
        {
            break;
        }
    }

    assert(arc_length_table_[i].arc_length_ > arc_length_table_[i-1].arc_length_);

    float arc_segment_length = arc_length_table_[i].arc_length_ - arc_length_table_[i-1].arc_length_;
    float interpolate_ratio = (arc_length - arc_length_table_[i-1].arc_length_) / arc_segment_length;

    return {arc_length_table_[i-1].segment_index_, arc_length_table_[i-1].segment_parameter_ + interpolate_ratio * 1.f/(ARC_LENGTH_SAMPLES-1)};
}

void Spline::printTable()
{
    for (size_t i = 0; i < arc_length_table_.size(); i++)
    {
        printf("%ld: %d - %f - %f\n", i, arc_length_table_.at(i).segment_index_, arc_length_table_.at(i).segment_parameter_, arc_length_table_.at(i).arc_length_);
    }
    printf("total: %f\n", total_length_);
}