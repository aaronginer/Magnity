#include "Level.h"
#include "cstdlib"

#define WIDTH 1200
#define HEIGTH 800

extern Level* current_level;

Level::Level(std::string name)
{
    this->name = name;
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
            // if it isn't also in the force sources vector (which it usually is), delete it
            if (std::find(p->force_sources.begin(), p->force_sources.end(), pa) == p->force_sources.end()) 
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

    auto pi_demo_button = tgui::Button::create();
    pi_demo_button->setText("PI Demo");
    pi_demo_button->setSize(100, 30);
    pi_demo_button->setOrigin(0.5f, 0.5f);
    pi_demo_button->setPosition({panel->getPosition().x-300, panel->getPosition().y+200});
    pi_demo_button->onClick([&window, &gui, &panel](){
        Level* c = current_level;
        c->destroy(window);
        gui.remove(c->level_panel_);
        current_level = LoadLevelPathInterpolDemo(window, gui);
    });

    panel->add(pi_demo_button);

    auto pd_demo_buttom = tgui::Button::create();
    pd_demo_buttom->setText("PD Demo");
    pd_demo_buttom->setSize(100, 30);
    pd_demo_buttom->setOrigin(0.5f, 0.5f);
    pd_demo_buttom->setPosition({panel->getPosition().x, panel->getPosition().y+200});
    pd_demo_buttom->onClick([&window, &gui, &panel](){
        Level* c = current_level;
        c->destroy(window);
        gui.remove(c->level_panel_);
        current_level = LoadLevelParticleDemo(window, gui);
    });

    panel->add(pd_demo_buttom);

    auto rb_demo_button = tgui::Button::create();
    rb_demo_button->setText("RB Demo");
    rb_demo_button->setSize(100, 30);
    rb_demo_button->setOrigin(0.5f, 0.5f);
    rb_demo_button->setPosition({panel->getPosition().x+300, panel->getPosition().y+200});
    rb_demo_button->onClick([&window, &gui, &panel](){
        // Level* c = current_level;
        // c->destroy(window);
        // gui.remove(c->level_panel_);
        // current_level = LoadLevelParticleDemo(window, gui);
    });

    panel->add(rb_demo_button);

    auto exit_button = tgui::Button::create();
    exit_button->setText("Exit Game");
    exit_button->setSize(100, 30);
    exit_button->setOrigin(0.5f, 0.5f);
    exit_button->setPosition({panel->getPosition().x, panel->getPosition().y+300});
    exit_button->onClick([&window, &gui, &panel](){
        Level* c = current_level;
        c->destroy(window);
        gui.remove(c->level_panel_);
        window.close();
    });

    panel->add(exit_button);

    gui.add(panel);

    // Textures
    sf::Texture* titleTexture = new sf::Texture();
    titleTexture->loadFromFile("res/title.png");

    // Sprites
    SpriteObject* title_sprite = new SpriteObject(*titleTexture, view.getCenter()+sf::Vector2f(0, -50));

    // create level
    Level* l = new Level("Level0");
    l->loaded_textures_.push_back(titleTexture);
    l->game_objects_.push_back(title_sprite);
    
    l->background_color_ = sf::Color::White;
    l->level_panel_ = panel;

    window.setView(view);
    return l;
}

Level* Level::LoadLevel1(sf::RenderWindow& window, tgui::GuiSFML& gui)
{
    View view(sf::FloatRect(0.f, 0.f, (float) WIDTH, (float) HEIGTH));

    // Textures
    sf::Texture* objectTexture = new sf::Texture();
    objectTexture->loadFromFile("res/object.png");

    // Splines

    // ParticleDynamics
    ParticleDynamics* pdyn = new ParticleDynamics(true);

    Particle* p = new Particle(*objectTexture, {WIDTH/2, HEIGTH/2}, {0, 0}, 10);
    p->sprite_->setScale({0.01f, 0.01f});
    pdyn->addParticle(p);
    pdyn->addForceSource(p);

    // RigidBodies

    // Magnets
    MagnetKeySet player1_keyset = { sf::Keyboard::Key::A, sf::Keyboard::Key::D, sf::Keyboard::Key::W, sf::Keyboard::Key::S, sf::Keyboard::Key::E };
    MagnetKeySet player2_keyset = { sf::Keyboard::Key::Left, sf::Keyboard::Key::Right, sf::Keyboard::Key::Up, sf::Keyboard::Key::Down, sf::Keyboard::Key::Enter };

    Magnet* m1 = new Magnet(player1_keyset, {40, 30}, 0);
    m1->setFollowObject(p->sprite_);
    Magnet* m2 = new Magnet(player2_keyset, {40, 500}, 1);
    m2->setFollowObject(p->sprite_);


    // create level
    Level* l = new Level("Level1");
    l->loaded_textures_.push_back(objectTexture);
    l->particle_dynamics_.push_back(pdyn);
    l->magnets_.push_back(m1);
    l->magnets_.push_back(m2);

    window.setView(view);
    return l;
}

Level* Level::LoadLevelParticleDemo(sf::RenderWindow& window, tgui::GuiSFML& gui)
{
    View view(sf::FloatRect(0.f, 0.f, (float) WIDTH, (float) HEIGTH));

    // Textures
    sf::Texture* objectTexture = new sf::Texture();
    objectTexture->loadFromFile("res/object.png");

    // Splines

    // ParticleDynamics
    ParticleDynamics* pdyn = new ParticleDynamics(true);

    for (int i = 0; i < 20; i++)
    {
        Particle* p = new Particle(*objectTexture, {std::rand() % WIDTH, std::rand() % HEIGTH}, {0, 0}, std::rand() % 50000);
        p->sprite_->setScale({0.01f, 0.01f});
        pdyn->addParticle(p);
        pdyn->addForceSource(p);
    }

    // RigidBodies

    // Magnets
    // MagnetKeySet player1_keyset = { sf::Keyboard::Key::A, sf::Keyboard::Key::D, sf::Keyboard::Key::W, sf::Keyboard::Key::S, sf::Keyboard::Key::E };
    // MagnetKeySet player2_keyset = { sf::Keyboard::Key::Left, sf::Keyboard::Key::Right, sf::Keyboard::Key::Up, sf::Keyboard::Key::Down, sf::Keyboard::Key::Enter };

    // Magnet* m1 = new Magnet(player1_keyset, {40, 30}, 0);
    // m1->setFollowObject(p_->sprite_);
    // Magnet* m2 = new Magnet(player2_keyset, {40, 500}, 1);
    // m2->setFollowObject(p_->sprite_);


    // create level
    Level* l = new Level("LevelPDDemo");
    l->loaded_textures_.push_back(objectTexture);

    l->particle_dynamics_.push_back(pdyn);

    window.setView(view);
    return l;
}

Level* Level::LoadLevelPathInterpolDemo(sf::RenderWindow& window, tgui::GuiSFML& gui)
{
    View view(sf::FloatRect(0.f, 0.f, (float) WIDTH, (float) HEIGTH));

    // Textures
    sf::Texture* objectTexture = new sf::Texture();
    objectTexture->loadFromFile("res/object.png");

    // Splines

    Spline* s = new Spline({{200, 200}, {200, HEIGTH-200}, {WIDTH-200, HEIGTH-200}, {WIDTH-200, 200}}, *objectTexture, true);
    Spline* s2 = new Spline({{400, 400}, {400, HEIGTH-400}, {WIDTH-400, HEIGTH-400}, {WIDTH-400, 400}}, *objectTexture, false);
    
    // ParticleDynamics

    // RigidBodies

    // Magnets

    // create level
    Level* l = new Level("LevelPIDemo");
    l->loaded_textures_.push_back(objectTexture);

    l->splines_.push_back(s);
    l->splines_.push_back(s2);;

    window.setView(view);
    return l;
}