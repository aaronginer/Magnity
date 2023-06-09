#include <SFML/Graphics.hpp>
#include "TGUI/TGUI.hpp"

#include <thread>
#include <chrono>
#include<unistd.h> 
#include <assert.h>
#include "PathInterpol.h"
#include "RigidBody.h"


#include <iostream>

#include "objects/Magnet.h"

#include "ParticleDynamics.h"
#include "Level.h"

using namespace std::chrono;

#define WIDTH 1200
#define HEIGTH 800
const float AR = (float)WIDTH/(float)HEIGTH;

extern void getClickedControlPoint(Level* level, sf::Vector2f mouse_position, sf::Vector2f** ctrl_point, sf::Sprite** ctrl_sprite, Spline** ctrl_spline);
tgui::Button::Ptr createControlsButton();
tgui::Button::Ptr createMenuButton(sf::RenderWindow& window, tgui::Gui& gui, tgui::Panel::Ptr panel);

tgui::Panel::Ptr createControlPanel(sf::RenderWindow& window);
tgui::Panel::Ptr createPausePanel(sf::RenderWindow& window, tgui::Gui& gui);
tgui::Panel::Ptr createWonPanel(sf::RenderWindow& window, tgui::Gui& gui);
tgui::Panel::Ptr createLostPanel(sf::RenderWindow& window, tgui::Gui& gui);
void updateControlPanelPosition(tgui::Panel::Ptr panel, sf::RenderWindow& window);
void updatePausePanelSize(tgui::Panel::Ptr panel, sf::RenderWindow& window);

sf::RenderWindow *mainWindow;
tgui::Panel::Ptr control_panel;
tgui::Button::Ptr control_button;
tgui::Panel::Ptr pause_panel;
tgui::Panel::Ptr won_panel;
tgui::Panel::Ptr lost_panel;

std::thread animation_loop_thread;

// Global variables
int animation_update_rate = 500;
int fps = 60;

float total_time = 0.0f;

bool control_panel_visible = false;
Level* current_level = nullptr;
bool game_paused = false;
bool won = false;
bool lost = false;

//Rigid Body Vector
std::vector<RigidBody*> *rigid_bodies = new std::vector<RigidBody*>;
//Borders / Fixed obstacles
std::vector<Border*> *obstacles = new std::vector<Border*>;

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
            sf::Texture& textureBody = const_cast<sf::Texture&>(*insertedBodies->at(i)->sprite_->sprite_.getTexture());
            textureBody.loadFromFile(insertedBodies->at(i)->nameImg);
            insertedBodies->at(i)->id = insertedBodies->at(i)->id + rigid_bodies->size();
            rigid_bodies->push_back(insertedBodies->at(i));
        }

        // subtract time needed for calculations
        usleep((1000000 / animation_update_rate) - clock.getElapsedTime().asMicroseconds());
    }
}

int main()
{
    mainWindow = new sf::RenderWindow(sf::VideoMode(WIDTH, HEIGTH), "Magnity!");
    (*mainWindow).setFramerateLimit(fps);

    // GUI
    tgui::GuiSFML gui(*mainWindow);

    control_panel = createControlPanel(*mainWindow);
    control_button = createControlsButton();
    pause_panel = createPausePanel(*mainWindow, gui);
    lost_panel = createLostPanel(*mainWindow, gui);
    won_panel = createWonPanel(*mainWindow, gui);
    gui.add(control_panel);
    gui.add(control_button);
    gui.add(pause_panel);
    gui.add(won_panel);
    gui.add(lost_panel);

    current_level = Level::LoadLevel0(*mainWindow, gui);

    animation_loop_thread = std::thread(animation_loop);

    float deltaTime = 0.0f;
    Clock clock;
    while ((*mainWindow).isOpen())
    {
        assert(current_level);

        deltaTime = clock.restart().asSeconds();
        sf::Vector2f mouse_pos = mainWindow->mapPixelToCoords(sf::Mouse::getPosition(*mainWindow)); // Mouse::getPosition(*mainWindow);

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
                    if (current_level->name.compare("Level0") != 0) game_paused = !game_paused;
                }

                current_level->handlePolledKeyInput(event);
                
            }
            else if (event.type == sf::Event::MouseButtonPressed && event.mouseButton.button == sf::Mouse::Left)
            {
                current_level->handleClick(mouse_pos);
            }
            else if (event.type == sf::Event::MouseButtonReleased && event.mouseButton.button == sf::Mouse::Left)
            {
                current_level->handleRelease();
            }
            else if (event.type == sf::Event::Resized)
            {
                // if (event.size.width < 800 || event.size.height < 600)
                // {
                //     mainWindow->setSize(sf::Vector2u(WIDTH, HEIGTH));
                // }
                mainWindow->setSize(sf::Vector2u(WIDTH, HEIGTH));
                updateControlPanelPosition(control_panel, *mainWindow);
                updatePausePanelSize(pause_panel, *mainWindow);
            }

            gui.handleEvent(event);
        }

        // instant input handling
        current_level->handleInstantKeyInput(deltaTime);
        current_level->updateMouseParticlePosition(mouse_pos);
        current_level->handleDrag(mouse_pos);

        // drawing
        (*mainWindow).clear(current_level == nullptr ? sf::Color::Black : current_level->background_color_);
        current_level->draw(*mainWindow, deltaTime);
        control_panel->setVisible(control_panel_visible);
        pause_panel->setVisible(game_paused && !(won || lost));
        won_panel->setVisible(won);
        lost_panel->setVisible(lost);
        control_button->setVisible(current_level->name.compare("Level0") != 0);
        gui.draw();

        (*mainWindow).display();
    }

    return 0;
}

tgui::Panel::Ptr createControlPanel(sf::RenderWindow& window)
{
    auto panel = tgui::Panel::create();
    panel->getRenderer()->setBackgroundColor(sf::Color(0, 0, 255, 128));

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

    tgui::Label::Ptr trailHistoryLabel = tgui::Label::create();
    trailHistoryLabel->setText("PI: Trail history:");
    trailHistoryLabel->setPosition(220, 260);
    trailHistoryLabel->setTextSize(16);

    tgui::Label::Ptr trailHistoryValueLabel = tgui::Label::create();
    trailHistoryValueLabel->setText(std::to_string(ParticleDynamics::trail_seconds));
    trailHistoryValueLabel->setPosition(380, 260);
    trailHistoryValueLabel->setTextSize(16);

    // Create the traversal speed slider and text field
    tgui::Slider::Ptr trailHistorySlider = tgui::Slider::create();
    trailHistorySlider->setMinimum(1);
    trailHistorySlider->setMaximum(10);
    trailHistorySlider->setStep(1);
    trailHistorySlider->setValue(ParticleDynamics::trail_seconds);
    trailHistorySlider->setSize(200, 20);
    trailHistorySlider->setPosition(10, 260);
    trailHistorySlider->onValueChange.connect([trailHistoryValueLabel](int value)
    { 
        ParticleDynamics::trail_seconds = value; 
        trailHistoryValueLabel->setText(std::to_string(ParticleDynamics::trail_seconds));
    });

    panel->add(trailHistoryLabel);
    panel->add(trailHistoryValueLabel);
    panel->add(trailHistorySlider);

    // Create the Draw Controls button
    tgui::Button::Ptr drawParticleFFButton = tgui::Button::create();
    drawParticleFFButton->setText("PD: Draw force-field");
    drawParticleFFButton->setSize(400, 30);
    drawParticleFFButton->setPosition(10, 290);
    drawParticleFFButton->onPress.connect([&]() { ParticleDynamics::draw_ff = !ParticleDynamics::draw_ff; });

    panel->add(drawParticleFFButton);

    tgui::Label::Ptr rk4CheckboxLabel = tgui::Label::create();
    rk4CheckboxLabel->setText("Use RK4: ");
    rk4CheckboxLabel->setPosition(10, 320);
    rk4CheckboxLabel->setTextSize(16);

    tgui::CheckBox::Ptr rk4Checkbox = tgui::CheckBox::create();
    rk4Checkbox->setChecked(true);
    rk4Checkbox->setPosition(80, 320);
    rk4Checkbox->onCheck([&](){
        ParticleDynamics::rk4 = true;
    });
    rk4Checkbox->onUncheck([&](){
        ParticleDynamics::rk4 = false;
    });

    panel->add(rk4CheckboxLabel);
    panel->add(rk4Checkbox);

    // panel sizing
    sf::Vector2f panelSize(0, 0);
    for (const auto& widget : panel->getWidgets())
    {
        panelSize.x = std::max(panelSize.x, widget->getPosition().x + widget->getSize().x);
        panelSize.y = std::max(panelSize.y, widget->getPosition().y + widget->getSize().y);
    }
    panel->setSize(tgui::Layout2d(panelSize.x, panelSize.y+5));
    updateControlPanelPosition(panel, window);

    return panel;
}

tgui::Panel::Ptr createPausePanel(sf::RenderWindow& window, tgui::Gui& gui)
{
    auto panel = tgui::Panel::create();
    panel->getRenderer()->setBackgroundColor(sf::Color(50, 50, 50, 128));
    panel->setSize({window.getSize().x, window.getSize().y});

    panel->add(createMenuButton(window, gui, panel));

    updatePausePanelSize(panel, window);

    return panel;
}

tgui::Panel::Ptr createWonPanel(sf::RenderWindow& window, tgui::Gui& gui)
{
    auto panel = tgui::Panel::create();
    panel->getRenderer()->setBackgroundColor(sf::Color(50, 50, 50, 128));
    panel->setSize({window.getSize().x, window.getSize().y});

    tgui::Label::Ptr won_label = tgui::Label::create();
    won_label->setText("Level Complete!");
    won_label->setOrigin(0.5f, 0.5f);
    won_label->setPosition({panel->getSize().x/2, panel->getSize().y/2-100});
    won_label->setTextSize(16);

    panel->add(won_label);
    panel->add(createMenuButton(window, gui, panel));

    updatePausePanelSize(panel, window);

    return panel;
}

tgui::Panel::Ptr createLostPanel(sf::RenderWindow& window, tgui::Gui& gui)
{
    auto panel = tgui::Panel::create();
    panel->getRenderer()->setBackgroundColor(sf::Color(50, 50, 50, 128));
    panel->setSize({window.getSize().x, window.getSize().y});

    tgui::Label::Ptr lost_label = tgui::Label::create();
    lost_label->setText("Level Failed!");
    lost_label->setOrigin(0.5f, 0.5f);
    lost_label->setPosition({panel->getSize().x/2, panel->getSize().y/2-100});
    lost_label->setTextSize(16);

    panel->add(lost_label);
    panel->add(createMenuButton(window, gui, panel));

    updatePausePanelSize(panel, window);

    return panel;
}

void updateControlPanelPosition(tgui::Panel::Ptr panel, sf::RenderWindow& window)
{
    // Position the panel widget at the top right of the window with a margin of 10 pixels    
    panel->setPosition(window.getSize().x - panel->getSize().x - 10, 10);
}

void updatePausePanelSize(tgui::Panel::Ptr panel, sf::RenderWindow& window)
{
    panel->setSize({window.getSize().x, window.getSize().y});
}

// creates the control button that toggles the controls gui
tgui::Button::Ptr createControlsButton()
{
     // Create the toggle button
    auto toggle_button = tgui::Button::create();
    toggle_button->setText("SaA Controls");
    toggle_button->setSize(100, 30);
    toggle_button->setPosition(10, 10);

    // Connect the button to the function that will toggle the GUI
    toggle_button->onClick.connect([&](){
        control_panel_visible = !control_panel_visible;
    });

    return toggle_button;
}

// creates a back to menu button
tgui::Button::Ptr createMenuButton(sf::RenderWindow& window, tgui::Gui& gui, tgui::Panel::Ptr panel)
{
    auto back_to_menu_button = tgui::Button::create();
    back_to_menu_button->setText("Menu");
    back_to_menu_button->setSize(100, 30);
    back_to_menu_button->setOrigin(0.5f, 0.5f);
    back_to_menu_button->setPosition({panel->getSize().x/2, panel->getSize().y/2});
    back_to_menu_button->onClick([&window, &gui, &panel](){
        Level* c = current_level;
        c->destroy(window);
        gui.remove(c->level_panel_);
        current_level = Level::LoadLevel0(window, gui);
        game_paused = false;
        won = false;
        lost = false;
    });

    return back_to_menu_button;
}