#include "WallArea.h"
#include "fstream"
#include "cstring"

WallArea::WallArea()
{}

WallArea::~WallArea()
{}


void WallArea::load(std::string file_path)
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
        // TODO: IF SOMETHING CRASHES
        sf::Texture* t = new sf::Texture();
        if (strcmp(type.c_str(), "warea_fixed") == 0)
        {
            float px, py, sx, sy;
            int floats_parsed = sscanf(rest.c_str(), "%f %f %f %f", &px, &py, &sx, &sy);
            
            RigidBody* wall_body = new RigidBody(10, 2.5, 3, sx, sy, *t, false,
                                        px + sx/2, py + sy/2, RigidBody::rigid_bodies->size());
            wall_body->visible = false;
            printf("SPAWNED FIXED: %d\n", wall_body->id);
            RigidBody::rigid_bodies->push_back(wall_body);

            sf::RectangleShape r({sx, sy});
            r.setPosition(px, py);
            r.setPosition(px, py);
            r.setFillColor(sf::Color(100, 100, 100, 150));
            r.setOutlineColor(sf::Color::Black);
            r.setOutlineThickness(2);

            this->areas_.push_back(r);
        }
        else if (strcmp(type.c_str(), "warea_bouncy") == 0)
        {
            float px, py, sx, sy;
            int floats_parsed = sscanf(rest.c_str(), "%f %f %f %f", &px, &py, &sx, &sy);
            
            RigidBody* wall_body = new RigidBody(10, 2.5, 2, sx, sy, *t, false,
                                        px + sx/2, py + sy/2, RigidBody::rigid_bodies->size());
            wall_body->visible = false;
            printf("SPAWNED BOUNCY: %d\n", wall_body->id);
            RigidBody::rigid_bodies->push_back(wall_body);

            sf::RectangleShape r({sx, sy});
            r.setPosition(px, py);
            r.setPosition(px, py);
            r.setFillColor(sf::Color(255, 50, 50, 150));
            r.setOutlineColor(sf::Color::Black);
            r.setOutlineThickness(2);

            this->areas_.push_back(r);
        }
    }
}

void WallArea::draw(sf::RenderWindow& window)
{
    for (sf::RectangleShape& r_draw : this->areas_)
    {
        window.draw(r_draw);
    }
}