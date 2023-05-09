#include "PathInterpol.h"
#include "cmath"

#define M_PI 3.14159265358979323846

extern sf::RenderWindow* mainWindow;

#define ARC_LENGTH_SAMPLES 30ULL

float distance(sf::Vector2f v1, sf::Vector2f v2)
{
    sf::Vector2f diff = v2-v1;
    return sqrt(pow(diff.x, 2) + pow(diff.y, 2));
}


SplineSegment::SplineSegment(sf::Vector2f p0, sf::Vector2f p1, sf::Vector2f p2, sf::Vector2f p3)
{
    ctrl_points_[0] = p0;
    ctrl_points_[1] = p1;
    ctrl_points_[2] = p2;
    ctrl_points_[3] = p3;

    initArcLengths();
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

void SplineSegment::initArcLengths()
{
    sf::Vector2f prev_point = getPoint(0);

    for (size_t i = 0; i < ARC_LENGTH_SAMPLES; i++)
    {
        sf::Vector2f next_point = getPoint((float)i/(ARC_LENGTH_SAMPLES-1));
        arc_points_.push_back(next_point);
        total_length_ += distance(prev_point, next_point);
        arc_lengths_.push_back(total_length_);

        prev_point = next_point;
    }
}

void SplineSegment::drawSamples()
{
    for (size_t i = 0; i < ARC_LENGTH_SAMPLES; i++)
    {
        sf::Vector2f point = getPoint((float)i/(ARC_LENGTH_SAMPLES-1));

        sf::CircleShape circle(.8f);
        circle.setFillColor(sf::Color::Yellow);
        circle.setPosition(point - sf::Vector2f(0.8f, 0.8f));
        (*mainWindow).draw(circle);
    }
}

Spline::Spline(std::vector<sf::Vector2f> ctrl_points)
{
    this->ctrl_points_ = ctrl_points;

    ctrl_texture_.loadFromFile("res/control_point.png");
    for (size_t i = 0; i < this->ctrl_points_.size(); i++)
    {
        sf::Sprite ctrl_sprite;
        ctrl_sprite.setTexture(ctrl_texture_);
        ctrl_sprite.setTextureRect({8, 8, 8, 8});
        ctrl_sprite.setPosition(ctrl_points_.at(i) - sf::Vector2f(4, 4));
        ctrl_sprites_.push_back(ctrl_sprite);
    }

    init();
}

void Spline::init()
{   
    mutex_.lock();
    
    this->arc_length_table_.clear();
    this->segments_.clear();
    this->total_length_ = 0;
    initSegments();
    initArcLengthTable();

    mutex_.unlock();
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
    mutex_.lock();

    time_ += time_delta * 1/traversal_speed_;
    double whole, t;
    t = modf(time_, &whole);

    if (easing_option_ == 1)
    {
        t = -(cos(M_PI * t) - 1) / 2; 
    }
    else if (easing_option_ == 2)
    {
        t = t < 0.5 ? 4 * t * t * t : 1 - pow(-2 * t + 2, 3) / 2;
    }
    
    std::pair<int, float> interpol = searchForU(total_length_ * t);
    current_pos_ = segments_[interpol.first].getPoint(interpol.second);
    
    mutex_.unlock();      
}

void Spline::drawObject()
{
    sf::CircleShape circle(7.0f);
    circle.setFillColor(sf::Color::Red);
    circle.setPosition(current_pos_ - sf::Vector2f(7, 7));

    (*mainWindow).draw(circle);
}

void Spline::drawControlPoints()
{
    if (!draw_ctrl_and_arc_) return;

    for (sf::Sprite& s : ctrl_sprites_)
    {
        (*mainWindow).draw(s);
    }
}

void Spline::drawArcSamples()
{
    if (!draw_ctrl_and_arc_) return;

    for (SplineSegment s : segments_)
    {
        s.drawSamples();
    }
}

#define DRAW_CURVE_SAMPLES_PER_UNIT 5
void Spline::drawCurve()
{
    if (!draw_curve_) return;

    size_t total_samples = (size_t)total_length_ * DRAW_CURVE_SAMPLES_PER_UNIT;
    for (size_t i = 0; i < total_samples; i++)
    {
        std::pair<int, float> interpol = searchForU((float)i / (total_samples-1) * total_length_);
        sf::Vector2f curve_position = segments_[interpol.first].getPoint(interpol.second);  

        sf::CircleShape circle(1.0f);
        circle.setFillColor(sf::Color::Green);
        circle.setPosition(curve_position - sf::Vector2f(1, 1));
        (*mainWindow).draw(circle);
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