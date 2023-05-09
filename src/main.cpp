#include <SFML/Graphics.hpp>
#include "TGUI/TGUI.hpp"

#include <thread>
#include <chrono>
#include<unistd.h> 
#include <assert.h>
#include "PathInterpol.h"


#include <iostream>
#include "Player.h"
#include "PlayerArea.h"
#include "Border.h"
#include "Object.h"
#include "Magnet.h"
#include "RigidBody.h"

using namespace std::chrono;

#define WIDTH 1200
#define HEIGTH 800
const float AR = (float)WIDTH/(float)HEIGTH;

sf::RenderWindow *mainWindow;

std::thread test_thread;


Spline s({{200, 200}, {250, 200}, {250, 500}, {500, 500}, {500, 200}, {550, 200}});

bool dragging_ctrl_point = false;
sf::Sprite* ctrl_point_to_drag;
size_t ctrl_point_to_drag_idx;

int animation_updates_per_second = 100;
int frames_per_second = 100;

RigidBody ball(1.0f, 1.0f, 1, 20.0f, 20.0f);

void animation_loop()
{
    float deltaTime = 0.0f;
    Clock clock;

    while((*mainWindow).isOpen())
    {
        deltaTime = clock.restart().asSeconds();

        // printf("time_delta: %f\n", delta_time.count()/1000.f);
        s.interpolate(deltaTime);
        ball.RunSimulation(deltaTime);

        usleep(1000000 / animation_updates_per_second);
    }
}

int main()
{
    mainWindow = new sf::RenderWindow(sf::VideoMode(WIDTH, HEIGTH), "Magnity!");
    (*mainWindow).setFramerateLimit(frames_per_second);
    View view(sf::FloatRect(0.f, 0.f, (float) WIDTH, (float) HEIGTH));
    mainWindow->setView(view);

    // GUI
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

    // Create the ComboBox and add the options to it
    auto comboBox = tgui::ComboBox::create();
    comboBox->setPosition(500, 10);
    comboBox->setSize(200, comboBox->getSize().y);
    comboBox->setDefaultText("Easing Off");
    comboBox->addItem("Easing Off");
    comboBox->addItem("Sin Easing");
    comboBox->addItem("Cubic Easing");

    comboBox->onItemSelect.connect([&](){
        s.easing_option_ = comboBox->getSelectedItemIndex();
    });
    gui.add(comboBox);

    // OBJECTS

    //------------------------------------- Textures -------------------------------------//
    Texture playerTexture;
    playerTexture.loadFromFile("res/animation.png");

    Texture playerAreaTexture;
    playerAreaTexture.loadFromFile("res/player_area.png");

    Texture borderTexture;
    borderTexture.loadFromFile("res/border.png");

    Texture objectTexture;
    objectTexture.loadFromFile("res/object.png");

    Texture magnetTexture;
    magnetTexture.loadFromFile("res/magnet.png");


    //------------------------------------- Objects  -------------------------------------//

    PlayerArea player1_area(&playerAreaTexture, view, 1);                                    //Player 1 Area
    PlayerArea player2_area(&playerAreaTexture, view, 2);                                    //Player 1 Area
    Border border_area1(&borderTexture, 0, player1_area.getArea().getSize().y, player1_area.getArea().getSize().x, 10.f); //Border player 1 area
    Border border_area2(&borderTexture, 0, view.getSize().y - player2_area.getArea().getSize().y, player1_area.getArea().getSize().x, 10.f); //Border player 2 area
    Object object(&objectTexture);                                                                    //Object
    
    ball.addToRigidBodies(&ball);

    Magnet magnet1(&magnetTexture, view, 1);                                                          //Player 1
    Magnet magnet2(&magnetTexture, view, 2);                                                          //Player 2


    test_thread = std::thread(animation_loop);

    float deltaTime = 0.0f;
    Clock clock;

    while ((*mainWindow).isOpen())
    {
        deltaTime = clock.restart().asSeconds();

        sf::Event event;
        while ((*mainWindow).pollEvent(event))
        {
            if (event.type == sf::Event::Closed)
            {
                (*mainWindow).close();
            }
            else if (event.type == sf::Event::MouseButtonPressed && event.mouseButton.button == sf::Mouse::Left)
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
            else if (event.type == sf::Event::MouseButtonReleased && event.mouseButton.button == sf::Mouse::Left)
            {
                dragging_ctrl_point = false;     
            }
            else if (event.type == sf::Event::Resized)
            {
                if (event.size.width < 800 || event.size.height < 600)
                {
                    mainWindow->setSize(sf::Vector2u(800, 600));
                }
            }

            gui.handleEvent(event);
        }

        if (dragging_ctrl_point)
        {
            sf::Vector2f mouse = mainWindow->mapPixelToCoords(sf::Mouse::getPosition(*mainWindow));
            ctrl_point_to_drag->setPosition(mouse - sf::Vector2f(4, 4));
            s.ctrl_points_[ctrl_point_to_drag_idx] = mouse;
            s.init();
        }

        //keyboard input player 1
        if(Keyboard::isKeyPressed(Keyboard::Key::A)) {
            magnet1.getMagnet().move(-1.f, 0.0);
            printf("--------------------------- (%f|%f)\n", magnet1.getPosition().x, magnet1.getPosition().y);
        }
        else if(Keyboard::isKeyPressed(Keyboard::Key::D)) {
            magnet1.getMagnet().move(1.f, 0.0);
        }
        else if(Keyboard::isKeyPressed(Keyboard::Key::S)) {
            magnet1.getMagnet().move(0.0, 1.f);
        }
        else if(Keyboard::isKeyPressed(Keyboard::Key::W)) {
            magnet1.getMagnet().move(0.0, -1.f);
        }

        //keyboard input player 2
        if(Keyboard::isKeyPressed(Keyboard::Key::Left)) {
            magnet2.getMagnet().move(-1.f, 0.0);
        }
        else if(Keyboard::isKeyPressed(Keyboard::Key::Right)) {
            magnet2.getMagnet().move(1.f, 0.0);
        }
        else if(Keyboard::isKeyPressed(Keyboard::Key::Down)) {
            magnet2.getMagnet().move(0.0, 1.f);
        }
        else if(Keyboard::isKeyPressed(Keyboard::Key::Up)) {
            magnet2.getMagnet().move(0.0, -1.f);
        }

        //(*mainWindow).setView(view);

        (*mainWindow).clear();

        

        ball.DisplayBodies(*mainWindow);

        player1_area.Draw(*mainWindow);
        player2_area.Draw(*mainWindow);
        border_area1.Draw(*mainWindow);
        border_area2.Draw(*mainWindow);
        object.Draw(*mainWindow);
        magnet1.Draw(*mainWindow);
        magnet2.Draw(*mainWindow);
        
        s.drawObject();
        s.drawCurve();
        s.drawControlPoints();
        s.drawArcSamples();
        gui.draw();

        (*mainWindow).display();
    }

    return 0;
}