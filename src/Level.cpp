#include "Level.h"

#define WIDTH 1200
#define HEIGTH 800

extern Level* current_level;
extern bool gui_visible;

tgui::Button::Ptr createControlsButton();

Level::Level()
{
}

void Level::destroy(sf::RenderWindow& window)
{
    for (Spline* s : this->splines_)
    {
        delete s;
    }

    for (ParticleDynamics* p : this->particle_dynamics_)
    {
        for (Particle* pa : p->particles)
        {
            delete pa;
        }
        for (ForceSource* f : p->force_sources)
        {
            delete f;
        }
        delete p;
    }

    /*for (RigidBody* r : this->rigid_bodies_)
    {
        delete r;
    }*/

    for (sf::Texture* t : loaded_textures_)
    {
        delete t;
    }

    for (Magnet* m : this->magnets_)
    {
        delete m;
    }

    for (GameObject* g : this->game_objects_)
    {
        delete g;
    }

    window.setView(window.getDefaultView());
}

void Level::update(float time_delta)
{
    for (Spline* s : this->splines_)
    {
        s->update(time_delta);
    }

    for (ParticleDynamics* p : this->particle_dynamics_)
    {
        p->update(time_delta);
    }

    /*for (RigidBody* r : this->rigid_bodies_)
    {
        // update rb
    }*/

    for (Magnet* m : this->magnets_)
    {
        m->updateRotation();
    }
}

void Level::updateMouseParticlePosition(sf::Vector2f new_pos)
{
    if (mouse_force == nullptr) return;
    mouse_force->x = new_pos;
}

void Level::draw(sf::RenderWindow& window, float delta_time)
{
    for (Spline* s : this->splines_)
    {
        s->drawCurve(window);
        s->drawArcSamples(window);
        s->drawControlPoints(window);
        s->drawObject(window);
    }

    for (ParticleDynamics* p : this->particle_dynamics_)
    {
        p->draw(window, delta_time);
        p->drawForceField(window);
        p->drawTrail(window);
    }
    

    /*for (RigidBody* r : this->rigid_bodies_)
    {
        // draw rb
    }*/

    for (GameObject* g : this->game_objects_)
    {
        g->draw(window);
    }

    for (Magnet* m : this->magnets_)
    {
        m->draw(window);
    }
}

void Level::handlePolledKeyInput(sf::Event keyEvent)
{
    for (Magnet* m : this->magnets_)
    {
        m->handlePolledKeyInput(keyEvent);
    }
}

void Level::handleInstantKeyInput(float delta_time)
{
    for (Magnet* m : this->magnets_)
    {
        m->handleInstantKeyInput(delta_time);
    }
}

void Level::handleClick(sf::Vector2f mouse_position)
{
    for (Spline* s : this->splines_)
    {
        s->handleClick(mouse_position);
    }
}

void Level::handleRelease()
{
    Spline::handleRelease();
}

void Level::handleDrag(sf::Vector2f mouse_position)
{
    Spline::handleDrag(mouse_position);
}


Level* Level::LoadLevel0(sf::RenderWindow& window, tgui::GuiSFML& gui)
{
    // view
    View view(sf::FloatRect(0.f, 0.f, (float) WIDTH, (float) HEIGTH));

    // gui
    tgui::Panel::Ptr panel = tgui::Panel::create();
    panel->setSize({view.getSize().x, view.getSize().y});
    panel->setOrigin({0.5f, 0.5f});
    panel->setPosition({view.getCenter().x, view.getCenter().y});
    panel->getRenderer()->setBackgroundColor(sf::Color(0, 0, 0, 0));

     // Create start game
    auto start_game_button = tgui::Button::create();
    start_game_button->setText("Start Game");
    start_game_button->setSize(100, 30);
    start_game_button->setOrigin(0.5f, 0.5f);
    start_game_button->setPosition({panel->getPosition().x, panel->getPosition().y+100});
    start_game_button->onClick([&window, &gui, &panel](){
        Level* c = current_level;
        c->destroy(window);
        gui.remove(c->level_panel_);
        current_level = LoadLevel1(window, gui);
    });

    panel->add(start_game_button);
    gui.add(panel);

    // Textures
    sf::Texture* titleTexture = new sf::Texture();
    titleTexture->loadFromFile("res/title.png");

    // Sprites
    SpriteObject* title_sprite = new SpriteObject(*titleTexture, view.getCenter()+sf::Vector2f(0, -50));

    // create level
    Level* l = new Level();
    l->loaded_textures_.push_back(titleTexture);
    l->game_objects_.push_back(title_sprite);
    
    l->background_color_ = sf::Color::White;
    l->level_panel_ = panel;

    window.setView(view);
    return l;
}

Level* Level::LoadLevel1(sf::RenderWindow& window, tgui::GuiSFML& gui)
{
    gui.add(createControlsButton());

    View view(sf::FloatRect(0.f, 0.f, (float) WIDTH, (float) HEIGTH));

    // Textures
    sf::Texture* objectTexture = new sf::Texture();
    objectTexture->loadFromFile("res/object.png");

    // Splines

    Spline* s = new Spline({{200, 200}, {200, HEIGTH-200}, {WIDTH-200, HEIGTH-200}, {WIDTH-200, 200}}, *objectTexture, true);
    Spline* s2 = new Spline({{400, 400}, {400, HEIGTH-400}, {WIDTH-400, HEIGTH-400}, {WIDTH-400, 400}}, *objectTexture, true);
    
    // ParticleDynamics
    ParticleDynamics* pdyn = new ParticleDynamics(true);

    Particle* p1 = new Particle(*objectTexture, view.getCenter()-sf::Vector2f(view.getCenter().x/2, 0), {0, 0}, 10);
    p1->sprite_->setScale({0.01f, 0.01f});
    ForceSource* f = new ForceSource(ForceType::AntiGravity, view.getCenter(), 50000);
    ForceSource* f1 = new ForceSource(ForceType::Gravity, view.getCenter(), 50000);
    ForceSource* f_c = new ForceSource(ForceType::Constant, sf::Vector2f(200, 0));

    pdyn->addParticle(p1);
    pdyn->addForceSource(f);   
    pdyn->addForceSource(f1);  
    // pdyn->addForceSource(f_c);  


    // RigidBodies

    // Magnets
    MagnetKeySet player1_keyset = { sf::Keyboard::Key::A, sf::Keyboard::Key::D, sf::Keyboard::Key::W, sf::Keyboard::Key::S, sf::Keyboard::Key::E };
    MagnetKeySet player2_keyset = { sf::Keyboard::Key::Left, sf::Keyboard::Key::Right, sf::Keyboard::Key::Up, sf::Keyboard::Key::Down, sf::Keyboard::Key::Enter };

    Magnet* m1 = new Magnet(player1_keyset, {40, 30}, 0);
    m1->setFollowObject(p1->sprite_);
    Magnet* m2 = new Magnet(player2_keyset, {40, 500}, 1);
    m2->setFollowObject(p1->sprite_);


    // create level
    Level* l = new Level();
    l->loaded_textures_.push_back(objectTexture);

    l->splines_.push_back(s);
    l->splines_.push_back(s2);
    l->particle_dynamics_.push_back(pdyn);
    l->magnets_.push_back(m1);
    l->magnets_.push_back(m2);
    
    l->mouse_force = f;

    window.setView(view);
    return l;
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
        gui_visible = !gui_visible;
    });

    return toggle_button;
}