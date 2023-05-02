#include <SFML/Graphics.hpp>
#include "TGUI/TGUI.hpp"

#include <thread>
#include <chrono>
#include<unistd.h> 
#include <assert.h>
#include "PathInterpol.h"

using namespace std::chrono;

sf::RenderWindow *mainWindow;

std::thread test_thread;


Spline s({{200, 200}, {250, 200}, {250, 500}, {500, 500}, {500, 200}, {550, 200}});

bool dragging_ctrl_point = false;
sf::Sprite* ctrl_point_to_drag;
size_t ctrl_point_to_drag_idx;

int animation_updates_per_second = 100;
int frames_per_second = 100;

void animation_loop()
{
    milliseconds time = duration_cast< milliseconds > ( system_clock::now().time_since_epoch() );
    while((*mainWindow).isOpen())
    {
        milliseconds c_time = duration_cast< milliseconds > ( system_clock::now().time_since_epoch() );
        milliseconds delta_time = c_time - time;
        time = c_time;

        // printf("time_delta: %f\n", delta_time.count()/1000.f);
        s.interpolate(delta_time.count()/1000.f);

        usleep(1000000 / animation_updates_per_second);
    }
}

int main()
{
    mainWindow = new sf::RenderWindow(sf::VideoMode(1280, 720), "SFML works!");
    (*mainWindow).setFramerateLimit(frames_per_second);

    tgui::GuiSFML gui(*mainWindow);
    auto text_traversal_speed = tgui::EditBox::create();
    text_traversal_speed->setPosition(230, 10);
    text_traversal_speed->setSize(200, text_traversal_speed->getSize().y);
    text_traversal_speed->setText("Traversal Speed: " + std::to_string(s.traversal_speed_));
    gui.add(text_traversal_speed);

    auto slider_traversal_speed = tgui::Slider::create(0.1f, 20);
    slider_traversal_speed->setPosition(10, 10);
    slider_traversal_speed->setSize(200, 10);
    slider_traversal_speed->setValue(4);
    slider_traversal_speed->setStep(0.1f);
    slider_traversal_speed->onValueChange.connect([&](float value) {
        // lock
        s.traversal_speed_ = value;
        text_traversal_speed->setText("Traversal Speed: " + std::to_string(value));
    });
    gui.add(slider_traversal_speed);

    auto text_aups = tgui::EditBox::create();
    text_aups->setPosition(230, 50);
    text_aups->setSize(200, text_traversal_speed->getSize().y);
    text_aups->setText("Animation Update Rate: " + std::to_string(animation_updates_per_second));
    gui.add(text_aups);

    auto slider_aups = tgui::Slider::create(1, 500);
    slider_aups->setPosition(10, 50);
    slider_aups->setSize(200, 10);
    slider_aups->setStep(1);
    slider_aups->setValue(animation_updates_per_second);
    slider_aups->onValueChange.connect([&](float value) {
        animation_updates_per_second = (int) value;
        text_aups->setText("Animation Update Rate: " + std::to_string((size_t)value));
    });
    gui.add(slider_aups);

    auto text_fps = tgui::EditBox::create();
    text_fps->setPosition(230, 90);
    text_fps->setSize(200, text_traversal_speed->getSize().y);
    text_fps->setText("FPS: " + std::to_string(frames_per_second));
    gui.add(text_fps);

    auto slider_fps = tgui::Slider::create(1, 500);
    slider_fps->setPosition(10, 90);
    slider_fps->setSize(200, 10);
    slider_fps->setStep(1);
    slider_fps->setValue(frames_per_second);
    slider_fps->onValueChange.connect([&](float value) {
        frames_per_second = (int) value;
        (*mainWindow).setFramerateLimit(frames_per_second);
        text_fps->setText("FPS: " + std::to_string((size_t)value));
    });
    gui.add(slider_fps);

    auto draw_curve_toggle = tgui::ToggleButton::create();
    draw_curve_toggle->onClick.connect([&]() { s.draw_curve_ = !s.draw_curve_; });
    draw_curve_toggle->setPosition(10, 110);
    draw_curve_toggle->setText("Draw Curve");
    gui.add(draw_curve_toggle);

    auto draw_ctrl_and_arc_toggle = tgui::ToggleButton::create();
    draw_ctrl_and_arc_toggle->onClick.connect([&]() { s.draw_ctrl_and_arc_ = !s.draw_ctrl_and_arc_; });
    draw_ctrl_and_arc_toggle->setPosition(10, 140);
    draw_ctrl_and_arc_toggle->setText("Draw Control Points and Arc-Length Table Samples");
    gui.add(draw_ctrl_and_arc_toggle);

    test_thread = std::thread(animation_loop);

    while ((*mainWindow).isOpen())
    {
        sf::Event event;
        while ((*mainWindow).pollEvent(event))
        {
            if (event.type == sf::Event::Closed)
            {
                (*mainWindow).close();
            }
            
            if (event.type == sf::Event::MouseButtonPressed && event.mouseButton.button == sf::Mouse::Left)
            {
                sf::Vector2f mouse = mainWindow->mapPixelToCoords(sf::Mouse::getPosition(*mainWindow));
                for (size_t i = 0; i < s.ctrl_sprites_.size(); i++)
                {
                    sf::FloatRect bounds = s.ctrl_sprites_[i].getGlobalBounds();

                    if (bounds.contains(mouse))
                    {
                        dragging_ctrl_point = true;
                        ctrl_point_to_drag = &s.ctrl_sprites_[i];
                        ctrl_point_to_drag_idx = i;
                    }
                }       
            }

            if (event.type == sf::Event::MouseButtonReleased && event.mouseButton.button == sf::Mouse::Left)
            {
                dragging_ctrl_point = false;     
            }

            gui.handleEvent(event);
        }

        if (dragging_ctrl_point)
        {
            sf::Vector2f mouse = mainWindow->mapPixelToCoords(sf::Mouse::getPosition(*mainWindow));
            ctrl_point_to_drag->setPosition(mouse);
            s.ctrl_points_[ctrl_point_to_drag_idx] = mouse;
            s.init();
        }

        (*mainWindow).clear();

        s.drawObject();
        s.drawCurve();
        s.drawControlPoints();

        gui.draw();
        (*mainWindow).display();
    }

    return 0;
}