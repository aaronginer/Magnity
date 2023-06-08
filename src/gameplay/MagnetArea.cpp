#include "MagnetArea.h"
#include "fstream"
#include "cstring"

MagnetArea::MagnetArea()
{}

MagnetArea::~MagnetArea()
{}


void MagnetArea::loadArea(std::string file_path)
{
    std::ifstream file(file_path);

    if (!file.is_open())
    {
        printf("An error occured opening the file. Exiting.\n");
        exit(-1);
    }

    std::string line;
    while (getline(file, line))
    {
        if (line.length() == 0)
        {
            continue;
        }
        int space_index = line.find(' ');
        std::string type = line.substr(0, space_index);
        std::string rest = line.substr(space_index+1, line.length()-(space_index+1));

        if (strcmp(type.c_str(), "spawn") == 0)
        {
            float x, y;
            int floats_parsed = sscanf(rest.c_str(), "%f %f", &x, &y);
            
            this->spawn_points_.push_back({x, y});
        }
        else if (strcmp(type.c_str(), "area") == 0)
        {
            float px, py, sx, sy;
            int floats_parsed = sscanf(rest.c_str(), "%f %f %f %f", &px, &py, &sx, &sy);
            
            #define MAGNET 70
            sf::RectangleShape r({sx-MAGNET, sy-MAGNET});
            r.setPosition(px+MAGNET/2, py+MAGNET/2);
            sf::RectangleShape r_draw({sx, sy});
            r_draw.setPosition(px, py);
            r_draw.setFillColor(sf::Color(0, 255, 0, 100));
            r_draw.setOutlineColor(sf::Color(0, 0, 0, 255));
            r_draw.setOutlineThickness(1);

            this->areas_.push_back(r);
            this->draw_areas_.push_back(r_draw);
        }
    }
}

void MagnetArea::draw(sf::RenderWindow& window)
{
    for (sf::RectangleShape& r_draw : this->draw_areas_)
    {
        window.draw(r_draw);
    }
}

bool MagnetArea::testLocationInBounds(sf::Vector2f location)
{
    for (sf::RectangleShape& r : this->areas_)
    {
        if (r.getGlobalBounds().contains(location))
        {
            return true;
        }
    }
    return false;
}

sf::Vector2f MagnetArea::getSpawnPoint()
{
    size_t points_available = this->spawn_points_.size();
    if (points_available-1 < this->magnets_) return {0, 0};

    return this->spawn_points_[this->magnets_++];
}