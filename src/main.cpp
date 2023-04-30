#include <SFML/Graphics.hpp>

#include <thread>
#include <chrono>
#include<unistd.h> 
#include <assert.h>
#include "PathInterpol.h"

using namespace std::chrono;

sf::RenderWindow *mainWindow;

std::thread test_thread;


Path p({{300, 30}, {100, 100}, {500, 500}, {300, 570}, {300, 500}, {200, 500}, {300, 30}, {100, 100}, {500, 500}});

void animation_loop()
{
    int update_rate = 3;

    milliseconds time = duration_cast< milliseconds > ( system_clock::now().time_since_epoch() );
    while((*mainWindow).isOpen())
    {
        milliseconds c_time = duration_cast< milliseconds > ( system_clock::now().time_since_epoch() );
        milliseconds delta_time = c_time - time;
        time = c_time;

        // printf("%ld\n", delta_time.count());

        p.interpolate(delta_time.count()/1000.f);

        usleep(1000000 / update_rate);
    }
}


int main()
{
    mainWindow = new sf::RenderWindow(sf::VideoMode(600, 600), "SFML works!");
    (*mainWindow).setFramerateLimit(120);

    test_thread = std::thread(animation_loop);

    while ((*mainWindow).isOpen())
    {
        sf::Event event;
        while ((*mainWindow).pollEvent(event))
        {
            if (event.type == sf::Event::Closed)
                (*mainWindow).close();
        }

        (*mainWindow).clear();

        p.drawObject();
        p.drawControlPoints();
        p.drawCurve();

        (*mainWindow).display();
    }

    return 0;
}