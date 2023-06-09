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

// Global variables
int animation_update_rate = 100;
int fps = 60;
int easing = 0;
bool gui_visible = true;

RigidBody* ball_ptr;

//Rigid Body Vector
std::vector<RigidBody*> *rigid_bodies = new std::vector<RigidBody*>;
//Borders / Fixed obstacles
std::vector<Border*> *obstacles = new std::vector<Border*>;

float total_time = 0.0f;


void createPathInterpolationPanel(tgui::Panel::Ptr panel, sf::RenderWindow& window)
{

    tgui::Label::Ptr traversalSpeedLabel = tgui::Label::create();
    traversalSpeedLabel->setText("Traversal Speed:");
    traversalSpeedLabel->setPosition(220, 10);
    traversalSpeedLabel->setTextSize(16);

    tgui::Label::Ptr traversalSpeedValueLabel = tgui::Label::create();
    traversalSpeedValueLabel->setText(std::to_string(s.traversal_speed_));
    traversalSpeedValueLabel->setPosition(380, 10);
    traversalSpeedValueLabel->setTextSize(16);

    // Create the traversal speed slider and text field
    tgui::Slider::Ptr traversalSpeedSlider = tgui::Slider::create();
    traversalSpeedSlider->setMinimum(0.1f);
    traversalSpeedSlider->setMaximum(20.0f);
    traversalSpeedSlider->setStep(0.05f);
    traversalSpeedSlider->setValue(s.traversal_speed_);
    traversalSpeedSlider->setSize(200, 20);
    traversalSpeedSlider->setPosition(10, 10);
    traversalSpeedSlider->onValueChange.connect([traversalSpeedValueLabel](float value)
    { 
        s.traversal_speed_ = value; 
        traversalSpeedValueLabel->setText(std::to_string(s.traversal_speed_));
    });

    panel->add(traversalSpeedSlider);
    panel->add(traversalSpeedLabel);
    panel->add(traversalSpeedValueLabel);


    tgui::Label::Ptr animationUpdateRateLabel = tgui::Label::create();
    animationUpdateRateLabel->setText("AUR:");
    animationUpdateRateLabel->setPosition(220, 40);
    animationUpdateRateLabel->setTextSize(16);

    tgui::Label::Ptr animationUpdateRateValueLabel = tgui::Label::create();
    animationUpdateRateValueLabel->setText(std::to_string(animation_update_rate));
    animationUpdateRateValueLabel->setPosition(380, 40);
    animationUpdateRateValueLabel->setTextSize(16);

    // Create the animation update rate slider and text field
    tgui::Slider::Ptr animationUpdateRateSlider = tgui::Slider::create();
    animationUpdateRateSlider->setMinimum(1.0f);
    animationUpdateRateSlider->setMaximum(500.0f);
    animationUpdateRateSlider->setStep(1.f);
    animationUpdateRateSlider->setValue(animation_update_rate);
    animationUpdateRateSlider->setSize(200, 20);
    animationUpdateRateSlider->setPosition(10, 40);
    animationUpdateRateSlider->onValueChange.connect([animationUpdateRateValueLabel](float value)
    { 
        animation_update_rate = (int) value;
        animationUpdateRateValueLabel->setText(std::to_string(animation_update_rate));
    });


    panel->add(animationUpdateRateSlider);
    panel->add(animationUpdateRateLabel);
    panel->add(animationUpdateRateValueLabel);

    tgui::Label::Ptr fpsLabel = tgui::Label::create();
    fpsLabel->setText("FPS:");
    fpsLabel->setPosition(220, 70);
    fpsLabel->setTextSize(16);

    tgui::Label::Ptr fpsValueLabel = tgui::Label::create();
    fpsValueLabel->setText(std::to_string((int) fps));
    fpsValueLabel->setPosition(380, 70);
    fpsValueLabel->setTextSize(16);

    // Create the fps slider and text field
    tgui::Slider::Ptr fpsSlider = tgui::Slider::create();
    fpsSlider->setMinimum(1.0f);
    fpsSlider->setMaximum(240.0f);
    fpsSlider->setStep(1.f);
    fpsSlider->setValue(fps);
    fpsSlider->setSize(200, 20);
    fpsSlider->setPosition(10, 70);
    fpsSlider->onValueChange.connect([fpsValueLabel](float value)
    { 
        fps = (int) value; 
        (*mainWindow).setFramerateLimit(fps);
        fpsValueLabel->setText(std::to_string(fps));
    });

    panel->add(fpsSlider);
    panel->add(fpsLabel);
    panel->add(fpsValueLabel);

    // Create the Draw Curve button
    tgui::Button::Ptr drawCurveButton = tgui::Button::create();
    drawCurveButton->setText("Draw Curve");
    drawCurveButton->setSize(200, 30);
    drawCurveButton->setPosition(10, 100);
    drawCurveButton->onPress.connect([&]() { s.draw_curve_ = !s.draw_curve_; });

    panel->add(drawCurveButton);

    // Create the Draw Controls button
    tgui::Button::Ptr drawControlsButton = tgui::Button::create();
    drawControlsButton->setText("Draw Control Points and Arc-Length Table samples");
    drawControlsButton->setSize(400, 30);
    drawControlsButton->setPosition(10, 140);
    drawControlsButton->onPress.connect([&]() { s.draw_ctrl_and_arc_ = !s.draw_ctrl_and_arc_; });

    panel->add(drawControlsButton);

    // Create the Easing dropdown menu
    tgui::ComboBox::Ptr easingComboBox = tgui::ComboBox::create();
    easingComboBox->addItem("Easing Off");
    easingComboBox->addItem("Sin Easing");
    easingComboBox->addItem("Cubic Easing");
    easingComboBox->setSize(200, 20);
    easingComboBox->setPosition(10, 180);
    easingComboBox->setDefaultText("Easing Off");
    easingComboBox->onItemSelect.connect([easingComboBox]{
        s.easing_option_ = easingComboBox->getSelectedItemIndex();
    });

    panel->add(easingComboBox);

    sf::Vector2f panelSize(0, 0);
    for (const auto& widget : panel->getWidgets())
    {
        panelSize.x = std::max(panelSize.x, widget->getPosition().x + widget->getSize().x);
        panelSize.y = std::max(panelSize.y, widget->getPosition().y + widget->getSize().y);
    }
    panel->setSize(tgui::Layout2d(panelSize.x, panelSize.y+5));

    // Position the panel widget at the top right of the window with a margin of 10 pixels
    panel->setPosition(window.getSize().x - panel->getSize().x - 10, 10);
}

void createToggleButtons(tgui::Panel::Ptr panel, tgui::Gui& gui)
{
     // Create the toggle button
    auto toggleButton = tgui::Button::create();
    toggleButton->setText("Toggle GUI");
    toggleButton->setSize(100, 30);
    toggleButton->setPosition(10, 10);

    // Add the button to the GUI
    gui.add(toggleButton);

    // Connect the button to the function that will toggle the GUI
    toggleButton->onClick.connect([&](){
        gui_visible = !gui_visible;
    });
}

void animation_loop()
{
    float deltaTime = 0.0f;
    Clock clock;

    while((*mainWindow).isOpen())
    {
        deltaTime = clock.restart().asSeconds();

        // printf("time_delta: %f\n", delta_time.count()/1000.f);
        s.interpolate(deltaTime);

        //Run Rigid Body simulation
        std::vector<RigidBody> *rigid_bodies_new = new std::vector<RigidBody>;
        for(int i = 0; i < rigid_bodies->size(); i++) {
            rigid_bodies_new->push_back(*rigid_bodies->at(i));
        }


        total_time += deltaTime;
        std::vector<RigidBody*> deleteBodies;
        std::vector<RigidBody*> *insertedBodies = new std::vector<RigidBody*>;
        RigidBody::ode(rigid_bodies, rigid_bodies_new,total_time - deltaTime,
                       total_time, rigid_bodies, *obstacles, insertedBodies);


        //delete rigid_bodies_new we don't need it anymore
        rigid_bodies_new = nullptr;
        delete rigid_bodies_new;

        //loop through bodies and delete or insert bodies
        for(int i = 0; i < insertedBodies->size(); i++) {
            sf::Texture& textureBody = const_cast<sf::Texture&>(*insertedBodies->at(i)->body.getTexture());
            textureBody.loadFromFile("/Users/laurapessl/Desktop/Magnity/macos/bin/" + insertedBodies->at(i)->nameImg);
            insertedBodies->at(i)->id = insertedBodies->at(i)->id + rigid_bodies->size();
            rigid_bodies->push_back(insertedBodies->at(i));
        }
    }
}

int main()
{
    mainWindow = new sf::RenderWindow(sf::VideoMode(WIDTH, HEIGTH), "Magnity!");
    (*mainWindow).setFramerateLimit(fps);
    View view(sf::FloatRect(0.f, 0.f, (float) WIDTH, (float) HEIGTH));
    mainWindow->setView(view);

    // GUI
    tgui::GuiSFML gui(*mainWindow);

    // Create a panel to serve as the background of the GUI
    tgui::Panel::Ptr panel = tgui::Panel::create();
    panel->setSize("100%", "100%");
    panel->getRenderer()->setBackgroundColor(sf::Color(0, 0, 255, 128));
    gui.add(panel);
    printf("%p\n", panel.get());

    createPathInterpolationPanel(panel, *mainWindow);
    createToggleButtons(panel, gui);

    // OBJECTS

    //------------------------------------- Textures -------------------------------------//
    Texture playerAreaTexture;
    playerAreaTexture.loadFromFile("/Users/laurapessl/Desktop/Magnity/res/player_area.png");

    Texture borderTexture;
    borderTexture.loadFromFile("/Users/laurapessl/Desktop/Magnity/res/border.png");

    Texture objectTexture2;
    objectTexture2.loadFromFile("/Users/laurapessl/Desktop/Magnity/res/object2.png");

    Texture objectTexture;
    objectTexture.loadFromFile("/Users/laurapessl/Desktop/Magnity/res/object.png");

    Texture magnetTexture;
    magnetTexture.loadFromFile("Users/laurapessl/Desktop/Magnity/res/magnet.png");


    //------------------------------------- Objects  -------------------------------------//

    PlayerArea player1_area(&playerAreaTexture, view, 1);                                    //Player 1 Area
    PlayerArea player2_area(&playerAreaTexture, view, 2);                                    //Player 1 Area
    Border border_area1(&borderTexture, 0, player1_area.getArea().getSize().y, player1_area.getArea().getSize().x, 10.f, 0); //Border player 1 area
    Border border_area2(&borderTexture, 0, view.getSize().y - player2_area.getArea().getSize().y, player1_area.getArea().getSize().x, 10.f, 2); //Border player 2 area
    obstacles->push_back(&border_area1);
    obstacles->push_back(&border_area2);

    Object object(&objectTexture);


    for(int i = 0; i < 1; i++) {
        RigidBody* ball = new RigidBody(1.0, 2.5, 0, 50.0, 50.0, objectTexture, false,
                                        206.0 - (i * 45), 350.0f - (i*50), rigid_bodies->size());
        rigid_bodies->push_back(ball);
    }

    for(int i = 0; i < 1; i++) {
        RigidBody* ball = new RigidBody(1.0, 2.5, 0, 20.0, 20.0, objectTexture2, false,
                                        223.0 + (i * 25), 400.0, rigid_bodies->size());
        rigid_bodies->push_back(ball);
    }

    Magnet magnet1(&magnetTexture, view, 1);                                                          //Player 1
    Magnet magnet2(&magnetTexture, view, 2);                                                          //Player 2


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
        if(Keyboard::isKeyPressed(Keyboard::Key::D)) {
            magnet1.getMagnet().move(1.f, 0.0);
        }
        if(Keyboard::isKeyPressed(Keyboard::Key::S)) {
            magnet1.getMagnet().move(0.0, 1.f);
        }
        if(Keyboard::isKeyPressed(Keyboard::Key::W)) {
            magnet1.getMagnet().move(0.0, -1.f);
        }

        //keyboard input player 2
        if(Keyboard::isKeyPressed(Keyboard::Key::Left)) {
            magnet2.getMagnet().move(-1.f, 0.0);
        }
        if(Keyboard::isKeyPressed(Keyboard::Key::Right)) {
            magnet2.getMagnet().move(1.f, 0.0);
        }
        if(Keyboard::isKeyPressed(Keyboard::Key::Down)) {
            magnet2.getMagnet().move(0.0, 1.f);
        }
        if(Keyboard::isKeyPressed(Keyboard::Key::Up)) {
            magnet2.getMagnet().move(0.0, -1.f);
        }

        //(*mainWindow).setView(view);

        (*mainWindow).clear();

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
        
        panel->setVisible(gui_visible);
        gui.draw();

        RigidBody::DisplayBodies(*mainWindow, rigid_bodies);
        (*mainWindow).display();
    }

    return 0;
}