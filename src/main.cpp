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
#include "ParticleDynamics.h"
#include "Level.h"

using namespace std::chrono;

#define WIDTH 1200
#define HEIGTH 800
const float AR = (float)WIDTH/(float)HEIGTH;

extern void getClickedControlPoint(Level* level, sf::Vector2f mouse_position, sf::Vector2f** ctrl_point, sf::Sprite** ctrl_sprite, Spline** ctrl_spline);

sf::RenderWindow *mainWindow;

std::thread animation_loop_thread;

sf::Vector2f* ctrl_point_to_drag = nullptr;
sf::Sprite* ctrl_sprite_to_drag = nullptr;
Spline* ctrl_spline_to_drag = nullptr;

// Global variables
int animation_update_rate = 500;
int fps = 60;
int easing = 0;
bool gui_visible = true;

Level* current_level = nullptr;
bool game_paused = false;

void createPathInterpolationPanel(tgui::Panel::Ptr panel, sf::RenderWindow& window)
{
    tgui::Label::Ptr traversalSpeedLabel = tgui::Label::create();
    traversalSpeedLabel->setText("PI: Traversal Speed:");
    traversalSpeedLabel->setPosition(220, 10);
    traversalSpeedLabel->setTextSize(16);

    tgui::Label::Ptr traversalSpeedValueLabel = tgui::Label::create();
    traversalSpeedValueLabel->setText(std::to_string(Spline::traversal_speed_));
    traversalSpeedValueLabel->setPosition(380, 10);
    traversalSpeedValueLabel->setTextSize(16);

    // Create the traversal speed slider and text field
    tgui::Slider::Ptr traversalSpeedSlider = tgui::Slider::create();
    traversalSpeedSlider->setMinimum(0.1f);
    traversalSpeedSlider->setMaximum(20.0f);
    traversalSpeedSlider->setStep(0.05f);
    traversalSpeedSlider->setValue(Spline::traversal_speed_);
    traversalSpeedSlider->setSize(200, 20);
    traversalSpeedSlider->setPosition(10, 10);
    traversalSpeedSlider->onValueChange.connect([traversalSpeedValueLabel](float value)
    { 
        Spline::traversal_speed_ = value; 
        traversalSpeedValueLabel->setText(std::to_string(Spline::traversal_speed_));
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
    drawCurveButton->setText("PI: Draw Curves");
    drawCurveButton->setSize(200, 30);
    drawCurveButton->setPosition(10, 100);
    drawCurveButton->onPress.connect([&]() { Spline::draw_curve_ = !Spline::draw_curve_; });

    panel->add(drawCurveButton);

    // Create the Draw Controls button
    tgui::Button::Ptr drawControlsButton = tgui::Button::create();
    drawControlsButton->setText("PI: Draw Control Points and Arc-Length Table samples");
    drawControlsButton->setSize(400, 30);
    drawControlsButton->setPosition(10, 140);
    drawControlsButton->onPress.connect([&]() { Spline::draw_ctrl_and_arc_ = !Spline::draw_ctrl_and_arc_; });

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
        Spline::easing_option_ = easingComboBox->getSelectedItemIndex();
    });

    panel->add(easingComboBox);

    // Create the Draw Controls button
    tgui::Button::Ptr drawParticleTrailsButton = tgui::Button::create();
    drawParticleTrailsButton->setText("PD: Draw trails");
    drawParticleTrailsButton->setSize(400, 30);
    drawParticleTrailsButton->setPosition(10, 220);
    drawParticleTrailsButton->onPress.connect([&]() { ParticleDynamics::draw_trails = !ParticleDynamics::draw_trails; });

    panel->add(drawParticleTrailsButton);

    // Create the Draw Controls button
    tgui::Button::Ptr drawParticleFFButton = tgui::Button::create();
    drawParticleFFButton->setText("PD: Draw force-field");
    drawParticleFFButton->setSize(400, 30);
    drawParticleFFButton->setPosition(10, 250);
    drawParticleFFButton->onPress.connect([&]() { ParticleDynamics::draw_ff = !ParticleDynamics::draw_ff; });

    panel->add(drawParticleFFButton);

    // panel sizing
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
        if (game_paused) {
            clock.restart().asSeconds();
            continue;
        }
        
        deltaTime = clock.restart().asSeconds();

        if (current_level != nullptr) current_level->update(deltaTime);

        // subtract time needed for calculations
        usleep((1000000 / animation_update_rate) - clock.getElapsedTime().asMicroseconds());
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

    createPathInterpolationPanel(panel, *mainWindow);
    createToggleButtons(panel, gui);
    
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

    Texture objectTexture2;
    objectTexture2.loadFromFile("res/object2.png");

    Texture magnetTexture;
    magnetTexture.loadFromFile("res/magnet.png");


    //------------------------------------- Objects  -------------------------------------//

    PlayerArea player1_area(&playerAreaTexture, view, 1);                                    //Player 1 Area
    PlayerArea player2_area(&playerAreaTexture, view, 2);                                    //Player 1 Area
    Border border_area1(&borderTexture, 0, player1_area.getArea().getSize().y, player1_area.getArea().getSize().x, 10.f); //Border player 1 area
    Border border_area2(&borderTexture, 0, view.getSize().y - player2_area.getArea().getSize().y, player1_area.getArea().getSize().x, 10.f); //Border player 2 area

    /*RigidBody ball(100.0f, 2.5f, 0, 20.0f, 20.0f, objectTexture, false,
                   206.0f, 206.0f);
    RigidBody ball2(100.0f, 2.5f, 1, 20.0f, 20.0f, objectTexture2, false,
                   300.0f, 306.0f);
    */

    Magnet magnet1(&magnetTexture, view, 1);                                                          //Player 1
    Magnet magnet2(&magnetTexture, view, 2);       

    current_level = Level::LoadLevel1(view);

    animation_loop_thread = std::thread(animation_loop);

    while ((*mainWindow).isOpen())
    {
        sf::Event event;
        while ((*mainWindow).pollEvent(event))
        {
            if (event.type == sf::Event::Closed)
            {
                (*mainWindow).close();
            }
            else if (event.type == sf::Event::KeyPressed)
            {
                if (event.key.code == Keyboard::Key::Escape)
                {
                    game_paused = !game_paused;
                }
            }
            else if (event.type == sf::Event::MouseButtonPressed && event.mouseButton.button == sf::Mouse::Left)
            {
                sf::Vector2f mouse = mainWindow->mapPixelToCoords(sf::Mouse::getPosition(*mainWindow));
                getClickedControlPoint(current_level, mouse, &ctrl_point_to_drag, &ctrl_sprite_to_drag, &ctrl_spline_to_drag); 
            }
            else if (event.type == sf::Event::MouseButtonReleased && event.mouseButton.button == sf::Mouse::Left)
            {
                ctrl_point_to_drag = nullptr;
                ctrl_sprite_to_drag = nullptr;
                ctrl_spline_to_drag = nullptr;     
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

        if (ctrl_point_to_drag != nullptr)
        {
            sf::Vector2f mouse = mainWindow->mapPixelToCoords(sf::Mouse::getPosition(*mainWindow));
            *ctrl_point_to_drag = mouse;
            ctrl_sprite_to_drag->setPosition(mouse - sf::Vector2f(4, 4));
            ctrl_spline_to_drag->init();
        }

        if (game_paused) continue;

        //keyboard input player 1
        if(Keyboard::isKeyPressed(Keyboard::Key::A)) {
            magnet1.getMagnet().move(-1.f, 0.0);
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

        (*mainWindow).clear();

        player1_area.Draw(*mainWindow);
        player2_area.Draw(*mainWindow);
        border_area1.Draw(*mainWindow);
        border_area2.Draw(*mainWindow);
        magnet1.Draw(*mainWindow);
        magnet2.Draw(*mainWindow);
        
        sf::Vector2i mousePos = Mouse::getPosition(*mainWindow);
        if (current_level != nullptr)
        {
            current_level->updateMouseParticlePosition({(float) mousePos.x, (float) mousePos.y});
            current_level->draw(*mainWindow);
        }

        panel->setVisible(gui_visible);
        gui.draw();

        // RigidBody::DisplayBodies(*mainWindow);
        (*mainWindow).display();
    }

    return 0;
}