#include "Level.h"
#include "cstdlib"
#include "cmath"
#include "background/LeafSpawner.h"

#define WIDTH 1200
#define HEIGTH 800

extern Level* current_level;
extern bool won;
extern bool lost;
extern bool game_paused;

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

    delete magnet_area_;
    delete wall_area_;
    delete target_area_;
    // delete object_;

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
        LeafSpawner::instance()->spawnLeaf(p);
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

    if (target_area_ != nullptr && object_ != nullptr)
    {
        if (target_area_->contains(object_->getPosition()))
        {
            won = true;
            game_paused = true;
        }

        sf::Vector2f v = sf::Vector2f(WIDTH/2, HEIGTH/2) - object_->getPosition();
        if (sqrt(pow(v.x, 2) + pow(v.y, 2)) >= 2000)
        {
            lost = true;
            game_paused = true;
        }

        target_area_->update(time_delta);
    }
}

void Level::updateMouseParticlePosition(sf::Vector2f new_pos)
{
    if (mouse_force == nullptr) return;
    mouse_force->x_ = new_pos;
}

void Level::draw(sf::RenderWindow& window, float delta_time)
{
    if (magnet_area_ != nullptr)
    {
        magnet_area_->draw(window);
    }

    if (wall_area_ != nullptr)
    {
        wall_area_->draw(window);
    }

    if (target_area_ != nullptr)
    {
        target_area_->draw(window);
    }

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
        m->handleInstantKeyInput(delta_time, this->magnet_area_);
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

// load the layout and positions of: magnet areas, magnets, target area, obstacles
/*void Level::loadFromFile(std::string file_name)
{

}*/

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
    auto level1_button = tgui::Button::create();
    level1_button->setText("Level 1");
    level1_button->setSize(100, 30);
    level1_button->setOrigin(0.5f, 0.5f);
    level1_button->setPosition({panel->getPosition().x-300, panel->getPosition().y+100});
    level1_button->onClick([&window, &gui, &panel](){
        Level* c = current_level;
        c->destroy(window);
        gui.remove(c->level_panel_);
        current_level = LoadLevel1(window, gui);
    });

    panel->add(level1_button);

    // Create start game
    auto level2_button = tgui::Button::create();
    level2_button->setText("Level 2");
    level2_button->setSize(100, 30);
    level2_button->setOrigin(0.5f, 0.5f);
    level2_button->setPosition({panel->getPosition().x, panel->getPosition().y+100});
    level2_button->onClick([&window, &gui, &panel](){
        Level* c = current_level;
        c->destroy(window);
        gui.remove(c->level_panel_);
        current_level = LoadLevel2(window, gui);
    });

    panel->add(level2_button);

    // Create start game
    auto level3_button = tgui::Button::create();
    level3_button->setText("Level 3");
    level3_button->setSize(100, 30);
    level3_button->setOrigin(0.5f, 0.5f);
    level3_button->setPosition({panel->getPosition().x+300, panel->getPosition().y+100});
    level3_button->onClick([&window, &gui, &panel](){
        Level* c = current_level;
        c->destroy(window);
        gui.remove(c->level_panel_);
        current_level = LoadLevel3(window, gui);
    });

    panel->add(level3_button);


    auto pi_demo_button = tgui::Button::create();
    pi_demo_button->setText("PI Demo");
    pi_demo_button->setSize(100, 30);
    pi_demo_button->setOrigin(0.5f, 0.5f);
    pi_demo_button->setPosition({panel->getPosition().x-300, panel->getPosition().y+150});
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
    pd_demo_buttom->setPosition({panel->getPosition().x, panel->getPosition().y+150});
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
    rb_demo_button->setPosition({panel->getPosition().x+300, panel->getPosition().y+150});
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
    exit_button->setPosition({panel->getPosition().x, panel->getPosition().y+200});
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
    sf::Texture* object_texture = new sf::Texture();
    object_texture->loadFromFile("res/object.png");
    sf::Texture* target_texture = new sf::Texture();
    target_texture->loadFromFile("res/target.png");

    // Splines

    // ParticleDynamics
    ParticleDynamics* pdyn = new ParticleDynamics(true);

    Particle* p = new Particle(*object_texture, {WIDTH/2, HEIGTH/2}, {0, 0}, 10);
    ForceSource* f_mouse = new ForceSource(ForceType::AntiGravity, {0, 0}, 10);

    ForceSource* f_m1 = new ForceSource(ForceType::Gravity, {0, 0}, 0);
    ForceSource* f_m2 = new ForceSource(ForceType::Gravity, {0, 0}, 0);
    ForceSource* f_g = new ForceSource(ForceType::Constant, {0, 981});

    p->sprite_->setScale({0.01f, 0.01f});
    pdyn->addParticle(p);
    pdyn->addForceSource(p);
    pdyn->addForceSource(f_mouse);
    pdyn->addForceSource(f_m1);
    pdyn->addForceSource(f_m2);
    pdyn->addForceSource(f_g);

    // RigidBodies

    // Magnets
    MagnetKeySet player1_keyset = { sf::Keyboard::Key::A, sf::Keyboard::Key::D, sf::Keyboard::Key::W, sf::Keyboard::Key::S, sf::Keyboard::Key::E };
    MagnetKeySet player2_keyset = { sf::Keyboard::Key::Left, sf::Keyboard::Key::Right, sf::Keyboard::Key::Up, sf::Keyboard::Key::Down, sf::Keyboard::Key::Enter };

    // Magnetarea
    MagnetArea* ma = new MagnetArea();
    ma->load("res/area_definitions/l1.txt");
    
    // Walls
    WallArea* wa = new WallArea();
    wa->load("res/area_definitions/l1.txt");

    // targetarea
    SpriteObject* ta = new SpriteObject(*target_texture, {500, 0}, 1);
    ta->setScale({0.2f, 0.2f});

    Magnet* m1 = new Magnet(player1_keyset, ma->getSpawnPoint(), 0);
    m1->setFollowObject(p->sprite_);
    m1->setForceSource(f_m1);
    Magnet* m2 = new Magnet(player2_keyset, ma->getSpawnPoint(), 1);
    m2->setFollowObject(p->sprite_);
    m2->setForceSource(f_m2);

    // create level
    Level* l = new Level("Level1");
    l->loaded_textures_.push_back(object_texture);
    l->loaded_textures_.push_back(target_texture);
    l->particle_dynamics_.push_back(pdyn);
    l->magnets_.push_back(m1);
    l->magnets_.push_back(m2);
    l->magnet_area_ = ma;
    l->wall_area_ = wa;
    l->target_area_ = ta;
    l->object_ = p->sprite_;

    l->mouse_force = f_mouse;

    window.setView(view);
    return l;
}

Level* Level::LoadLevel2(sf::RenderWindow& window, tgui::GuiSFML& gui)
{
    View view(sf::FloatRect(0.f, 0.f, (float) WIDTH, (float) HEIGTH));

    // Textures
    sf::Texture* object_texture = new sf::Texture();
    object_texture->loadFromFile("res/object.png");
    sf::Texture* target_texture = new sf::Texture();
    target_texture->loadFromFile("res/target.png");

    // Splines

    // ParticleDynamics
    ParticleDynamics* pdyn = new ParticleDynamics(true);

    Particle* p = new Particle(*object_texture, {0, HEIGTH}, {0, 0}, 10);
    ForceSource* f_mouse = new ForceSource(ForceType::AntiGravity, {0, 0}, 10);

    ForceSource* f_m1 = new ForceSource(ForceType::Gravity, {0, 0}, 0);
    ForceSource* f_m2 = new ForceSource(ForceType::Gravity, {0, 0}, 0);
    ForceSource* f_g = new ForceSource(ForceType::Constant, {0, 981});

    p->sprite_->setScale({0.01f, 0.01f});
    pdyn->addParticle(p);
    pdyn->addForceSource(p);
    pdyn->addForceSource(f_mouse);
    pdyn->addForceSource(f_m1);
    pdyn->addForceSource(f_m2);
    pdyn->addForceSource(f_g);

    // RigidBodies

    // Magnets
    MagnetKeySet player1_keyset = { sf::Keyboard::Key::A, sf::Keyboard::Key::D, sf::Keyboard::Key::W, sf::Keyboard::Key::S, sf::Keyboard::Key::E };
    MagnetKeySet player2_keyset = { sf::Keyboard::Key::Left, sf::Keyboard::Key::Right, sf::Keyboard::Key::Up, sf::Keyboard::Key::Down, sf::Keyboard::Key::Enter };

    // Magnetarea
    MagnetArea* ma = new MagnetArea();
    ma->load("res/area_definitions/l1.txt");

    // targetarea
    Spline* ta = new Spline({{500, 100}, {500, 100}, {500, 700}, {500, 700}}, *target_texture, true);
    ta->setOrigin(1);
    ta->setScale({0.2f, 0.2f});

    Magnet* m1 = new Magnet(player1_keyset, ma->getSpawnPoint(), 0);
    m1->setFollowObject(p->sprite_);
    m1->setForceSource(f_m1);
    Magnet* m2 = new Magnet(player2_keyset, ma->getSpawnPoint(), 1);
    m2->setFollowObject(p->sprite_);
    m2->setForceSource(f_m2);

    // create level
    Level* l = new Level("Level2");
    l->loaded_textures_.push_back(object_texture);
    l->loaded_textures_.push_back(target_texture);
    l->particle_dynamics_.push_back(pdyn);
    l->magnets_.push_back(m1);
    l->magnets_.push_back(m2);
    l->magnet_area_ = ma;
    l->target_area_ = ta;
    l->object_ = p->sprite_;

    l->mouse_force = f_mouse;

    window.setView(view);
    return l;
}

Level* Level::LoadLevel3(sf::RenderWindow& window, tgui::GuiSFML& gui)
{
    LeafSpawner::instance()->enable();
    View view(sf::FloatRect(0.f, 0.f, (float) WIDTH, (float) HEIGTH));

    // Textures

    // Splines

    // ParticleDynamics
    ParticleDynamics* pdyn = new ParticleDynamics(true);

    ForceSource* f_mouse = new ForceSource(ForceType::AntiGravity, {0, 0}, 500000);
    ForceSource* f_c = new ForceSource(ForceType::Constant, {5000, 0});
    ForceSource* f_g = new ForceSource(ForceType::Constant, {0, 981});

    for (int i = 0; i < 10; i++)
    {   
        sf::Vector2f pos = {std::rand() % WIDTH, std::rand() % HEIGTH};
        ForceSource* f = new ForceSource(ForceType::Gravity, pos, 20000 + std::rand() % 400000);
        pdyn->addForceSource(f);
    }
    pdyn->addForceSource(f_mouse);
    pdyn->addForceSource(f_c);
    pdyn->addForceSource(f_g);

    // RigidBodies

    // Magnets

    // Magnetarea

    // targetarea

    // create level
    Level* l = new Level("Level3");
    l->particle_dynamics_.push_back(pdyn);

    l->mouse_force = f_mouse;

    window.setView(view);
    return l;
}

Level* Level::LoadLevelWindTest(sf::RenderWindow& window, tgui::GuiSFML& gui)
{
    View view(sf::FloatRect(0.f, 0.f, (float) WIDTH, (float) HEIGTH));

    // Textures
    sf::Texture* object_texture = new sf::Texture();
    object_texture->loadFromFile("res/object.png");

    // Splines

    // ParticleDynamics
    ParticleDynamics* pdyn = new ParticleDynamics(true);

    Particle* p = new Particle(*object_texture, {0, WIDTH/2}, {0, 0}, std::rand() % 1000);
    ForceSource* f_g = new ForceSource(ForceType::Constant, {0, 981});
    ForceSource* f_wind = new ForceSource(ForceType::VectorField, VectorFieldFunction::Wind);
    p->K_ = 2;
    p->sprite_->setScale({0.01f, 0.01f});
    pdyn->addParticle(p);
    pdyn->addForceSource(f_g);
    pdyn->addForceSource(f_wind);

    // RigidBodies

    // Magnets

    // Magnetarea

    // create level
    Level* l = new Level("LevelPDDemo");
    l->loaded_textures_.push_back(object_texture);

    l->particle_dynamics_.push_back(pdyn);

    window.setView(view);
    return l;
}

Level* Level::LoadLevelParticleDemo(sf::RenderWindow& window, tgui::GuiSFML& gui)
{
    View view(sf::FloatRect(0.f, 0.f, (float) WIDTH, (float) HEIGTH));

    // Textures
    sf::Texture* object_texture = new sf::Texture();
    object_texture->loadFromFile("res/object.png");

    // Splines

    // ParticleDynamics
    ParticleDynamics* pdyn = new ParticleDynamics(true);

    for (int i = 0; i < 20; i++)
    {
        Particle* p = new Particle(*object_texture, {std::rand() % WIDTH, std::rand() % HEIGTH}, {0, 0}, std::rand() % 50000);
        p->sprite_->setScale({0.01f, 0.01f});
        pdyn->addParticle(p);
        pdyn->addForceSource(p);
    }

    // RigidBodies

    // Magnets

    // Magnetarea

    // create level
    Level* l = new Level("LevelPDDemo");
    l->loaded_textures_.push_back(object_texture);

    l->particle_dynamics_.push_back(pdyn);

    window.setView(view);
    return l;
}

Level* Level::LoadLevelPathInterpolDemo(sf::RenderWindow& window, tgui::GuiSFML& gui)
{
    View view(sf::FloatRect(0.f, 0.f, (float) WIDTH, (float) HEIGTH));

    // Textures
    sf::Texture* object_texture = new sf::Texture();
    object_texture->loadFromFile("res/object.png");

    // Splines

    Spline* s = new Spline({{200, 200}, {200, HEIGTH-200}, {WIDTH-200, HEIGTH-200}, {WIDTH-200, 200}}, *object_texture, true);
    Spline* s2 = new Spline({{400, 400}, {400, HEIGTH-400}, {WIDTH-400, HEIGTH-400}, {WIDTH-400, 400}}, *object_texture, false);
    
    // ParticleDynamics

    // RigidBodies

    // Magnets

    // Magnetarea

    // create level
    Level* l = new Level("LevelPIDemo");
    l->loaded_textures_.push_back(object_texture);

    l->splines_.push_back(s);
    l->splines_.push_back(s2);

    l->background_color_ = sf::Color::Black;

    window.setView(view);
    return l;
}