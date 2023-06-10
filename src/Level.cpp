#include "Level.h"
#include "cstdlib"
#include "cmath"
#include "background/SpriteSpawner.h"
#include "mutex"

#define WIDTH 1200
#define HEIGTH 800

extern Level* current_level;
extern Level* (*currentLevelFunction)(sf::RenderWindow& window, tgui::GuiSFML& gui);
extern std::mutex level_lock;
extern bool won;
extern bool lost;
extern bool game_paused;

Level::Level(std::string name)
{
    this->name = name;
    this->level_complete_sound_.openFromFile("res/audio/level_complete.wav");
    this->level_failed_sound_.openFromFile("res/audio/level_failed.wav");
}

void Level::destroy(sf::RenderWindow& window)
{
    assert(level_lock.try_lock() == false);

    for (Spline* s : this->splines_)
    {
        if (s == target_area_) continue;
        delete s;
    }

    auto pdyn_iter = this->particle_dynamics_.begin();
    for (; pdyn_iter != this->particle_dynamics_.end(); )
    {
        for (Particle* pa : (*pdyn_iter)->particles)
        {
            // if it isn't also in the force sources vector (which it usually is), delete it
            if (std::find((*pdyn_iter)->force_sources.begin(), (*pdyn_iter)->force_sources.end(), pa) == (*pdyn_iter)->force_sources.end()) 
                delete pa;
        }
        for (ForceSource* f : (*pdyn_iter)->force_sources)
        {
            delete f;
        }
        delete *pdyn_iter;
        this->particle_dynamics_.erase(pdyn_iter);
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

    this->background_music_.stop();

    window.setView(window.getDefaultView());
}

void Level::update(float time_delta)
{
    level_lock.lock();

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

    if (target_area_ != nullptr && object_ != nullptr)
    {
        if (target_area_->contains(object_->getPosition()))
        {
            level_complete_sound_.play();
            won = true;
            game_paused = true;
        }

        sf::Vector2f v = sf::Vector2f(WIDTH/2, HEIGTH/2) - object_->getPosition();
        if (sqrt(pow(v.x, 2) + pow(v.y, 2)) >= 2000)
        {
            level_failed_sound_.play();
            lost = true;
            game_paused = true;
        }

        target_area_->update(time_delta);
    }

    level_lock.unlock();
}

void Level::updateMouseParticlePosition(sf::Vector2f new_pos)
{
    if (mouse_force == nullptr) return;
    mouse_force->x_ = new_pos;
}

void Level::draw(sf::RenderWindow& window, float delta_time)
{
    level_lock.lock();

    for (GameObject* g : this->game_objects_)
    {
        g->draw(window);
    }

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
        if (s != target_area_) s->drawObject(window);
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

    for (Magnet* m : this->magnets_)
    {
        m->draw(window);
    }

    level_lock.unlock();
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
    SpriteSpawner::disable();

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
    level1_button->setPosition({panel->getPosition().x-400, panel->getPosition().y+100});
    level1_button->onClick([&window, &gui, &panel](){
        LoadLevel(window, gui, &LoadLevel1);
    });

    panel->add(level1_button);

    // Create start game
    auto level2_button = tgui::Button::create();
    level2_button->setText("Level 2");
    level2_button->setSize(100, 30);
    level2_button->setOrigin(0.5f, 0.5f);
    level2_button->setPosition({panel->getPosition().x-150, panel->getPosition().y+100});
    level2_button->onClick([&window, &gui, &panel](){
        LoadLevel(window, gui, &LoadLevel2);
    });

    panel->add(level2_button);

    // Create start game
    auto level3_button = tgui::Button::create();
    level3_button->setText("Level 3");
    level3_button->setSize(100, 30);
    level3_button->setOrigin(0.5f, 0.5f);
    level3_button->setPosition({panel->getPosition().x+150, panel->getPosition().y+100});
    level3_button->onClick([&window, &gui, &panel](){
        LoadLevel(window, gui, &LoadLevel3);
    });

    panel->add(level3_button);

    // Create start game
    auto level4_button = tgui::Button::create();
    level4_button->setText("Level 4");
    level4_button->setSize(100, 30);
    level4_button->setOrigin(0.5f, 0.5f);
    level4_button->setPosition({panel->getPosition().x+400, panel->getPosition().y+100});
    level4_button->onClick([&window, &gui, &panel](){
        LoadLevel(window, gui, &LoadLevel4);
    });

    panel->add(level4_button);


    auto pi_demo_button = tgui::Button::create();
    pi_demo_button->setText("PI Demo");
    pi_demo_button->setSize(100, 30);
    pi_demo_button->setOrigin(0.5f, 0.5f);
    pi_demo_button->setPosition({panel->getPosition().x-400, panel->getPosition().y+150});
    pi_demo_button->onClick([&window, &gui, &panel](){
        LoadLevel(window, gui, &LoadLevelPathInterpolDemo);
    });

    panel->add(pi_demo_button);

    auto pd_demo_buttom = tgui::Button::create();
    pd_demo_buttom->setText("PD Demo");
    pd_demo_buttom->setSize(100, 30);
    pd_demo_buttom->setOrigin(0.5f, 0.5f);
    pd_demo_buttom->setPosition({panel->getPosition().x-150, panel->getPosition().y+150});
    pd_demo_buttom->onClick([&window, &gui, &panel](){
        LoadLevel(window, gui, &LoadLevelParticleDemo);
    });

    panel->add(pd_demo_buttom);

    auto rb_demo_button = tgui::Button::create();
    rb_demo_button->setText("RB Demo");
    rb_demo_button->setSize(100, 30);
    rb_demo_button->setOrigin(0.5f, 0.5f);
    rb_demo_button->setPosition({panel->getPosition().x+150, panel->getPosition().y+150});
    rb_demo_button->onClick([&window, &gui, &panel](){
        // LoadLevel(window, gui, &LoadLevelRigidBodyDemo);
    });

    panel->add(rb_demo_button);

    auto vf_demo_button = tgui::Button::create();
    vf_demo_button->setText("VF Demo");
    vf_demo_button->setSize(100, 30);
    vf_demo_button->setOrigin(0.5f, 0.5f);
    vf_demo_button->setPosition({panel->getPosition().x+400, panel->getPosition().y+150});
    vf_demo_button->onClick([&window, &gui, &panel](){
        // LoadLevel(window, gui, &LoadLevelVoronoiDemo);
    });

    panel->add(vf_demo_button);

    auto exit_button = tgui::Button::create();
    exit_button->setText("Exit Game");
    exit_button->setSize(100, 30);
    exit_button->setOrigin(0.5f, 0.5f);
    exit_button->setPosition({panel->getPosition().x, panel->getPosition().y+200});
    exit_button->onClick([&window, &gui, &panel](){
        level_lock.lock();
        Level* c = current_level;
        c->destroy(window);
        gui.remove(c->level_panel_);
        window.close();
        level_lock.unlock();
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
    sf::Texture* bg_texture = new sf::Texture();
    bg_texture->loadFromFile("res/spring.jpg");

    // background
    SpriteObject* background = new SpriteObject(*bg_texture, view.getCenter(), 0);
    background->setScale({0.5f, 0.5f});

    // Splines

    // ParticleDynamics
    ParticleDynamics* pdyn = new ParticleDynamics(true);

    Particle* p = new Particle(*object_texture, {WIDTH/2, HEIGTH/2}, {0, 0}, 10);
    p->sprite_->setScale({1, 1});
    ForceSource* f_mouse = new ForceSource(ForceType::AntiGravity, {0, 0}, 10);

    ForceSource* f_m1 = new ForceSource(ForceType::Gravity, {0, 0}, 0);
    ForceSource* f_m2 = new ForceSource(ForceType::Gravity, {0, 0}, 0);
    ForceSource* f_g = new ForceSource(ForceType::Constant, {0, 981});

    pdyn->addParticle(p);
    pdyn->addForceSource(p);
    pdyn->addForceSource(f_mouse);
    pdyn->addForceSource(f_m1);
    pdyn->addForceSource(f_m2);
    pdyn->addForceSource(f_g);

    // RigidBodies

    
    // Magnetarea
    MagnetArea* ma = new MagnetArea();
    ma->load("res/area_definitions/l1.txt");
    
    // Walls
    WallArea* wa = new WallArea();
    wa->load("res/area_definitions/l1.txt");

    // targetarea
    SpriteObject* ta = new SpriteObject(*target_texture, {500, 0}, 1);
    ta->setScale({0.2f, 0.2f});
    
    // Magnets
    MagnetKeySet player1_keyset = { sf::Keyboard::Key::A, sf::Keyboard::Key::D, sf::Keyboard::Key::W, sf::Keyboard::Key::S, sf::Keyboard::Key::E };
    MagnetKeySet player2_keyset = { sf::Keyboard::Key::Left, sf::Keyboard::Key::Right, sf::Keyboard::Key::Up, sf::Keyboard::Key::Down, sf::Keyboard::Key::Enter };

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
    l->loaded_textures_.push_back(bg_texture);
    l->particle_dynamics_.push_back(pdyn);
    l->game_objects_.push_back(background);
    l->magnets_.push_back(m1);
    l->magnets_.push_back(m2);
    l->magnet_area_ = ma;
    l->wall_area_ = wa;
    l->target_area_ = ta;
    l->object_ = p->sprite_;

    l->mouse_force = f_mouse;

    if (l->background_music_.openFromFile("res/audio/spring.wav"))
    {
        l->background_music_.setVolume(20);
        l->background_music_.play();
        l->background_music_.setLoop(true);
    }

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
    sf::Texture* bg_texture = new sf::Texture();
    bg_texture->loadFromFile("res/summer.jpg");

    // background
    SpriteObject* background = new SpriteObject(*bg_texture, view.getCenter(), 0);
    background->setScale({0.5f, 0.5f});

    // Splines

    // ParticleDynamics
    ParticleDynamics* pdyn = new ParticleDynamics(true);

    Particle* p = new Particle(*object_texture, {WIDTH/2, HEIGTH/2}, {0, 0}, 10);
    p->sprite_->setScale({1, 1});

    ForceSource* f_mouse = new ForceSource(ForceType::AntiGravity, {0, 0}, 10);

    ForceSource* f_m1 = new ForceSource(ForceType::Gravity, {0, 0}, 0);
    ForceSource* f_m2 = new ForceSource(ForceType::Gravity, {0, 0}, 0);
    ForceSource* f_g = new ForceSource(ForceType::Constant, {0, 981});

    pdyn->addParticle(p);
    pdyn->addForceSource(p);
    pdyn->addForceSource(f_mouse);
    pdyn->addForceSource(f_m1);
    pdyn->addForceSource(f_m2);
    pdyn->addForceSource(f_g);

    // RigidBodies

    // Magnetarea
    MagnetArea* ma = new MagnetArea();
    ma->load("res/area_definitions/l2.txt");

    // targetarea
    Spline* ta = new Spline({{300, 100}, {300, 100}, {900, 100}, {900, 100}}, *target_texture, true);
    ta->setOrigin(0);
    ta->setScale({0.2f, 0.2f});

    // Walls
    WallArea* wa = new WallArea();
    wa->load("res/area_definitions/l2.txt");
    
    // Magnets
    MagnetKeySet player1_keyset = { sf::Keyboard::Key::A, sf::Keyboard::Key::D, sf::Keyboard::Key::W, sf::Keyboard::Key::S, sf::Keyboard::Key::E };
    MagnetKeySet player2_keyset = { sf::Keyboard::Key::Left, sf::Keyboard::Key::Right, sf::Keyboard::Key::Up, sf::Keyboard::Key::Down, sf::Keyboard::Key::Enter };

    Magnet* m1 = new Magnet(player1_keyset, ma->getSpawnPoint(), 0);
    m1->setFollowObject(p->sprite_);
    m1->setForceSource(f_m1);
    Magnet* m2 = new Magnet(player2_keyset, ma->getSpawnPoint(), 1);
    m2->setFollowObject(p->sprite_);
    m2->setForceSource(f_m2);

    // create level
    Level* l = new Level("Level2");
    l->splines_.push_back(ta);
    l->loaded_textures_.push_back(object_texture);
    l->loaded_textures_.push_back(target_texture);
    l->loaded_textures_.push_back(bg_texture);
    l->particle_dynamics_.push_back(pdyn);
    l->magnets_.push_back(m1);
    l->magnets_.push_back(m2);
    l->game_objects_.push_back(background);
    l->magnet_area_ = ma;
    l->target_area_ = ta;
    l->wall_area_ = wa;
    l->object_ = p->sprite_;

    l->mouse_force = f_mouse;

    if (l->background_music_.openFromFile("res/audio/spring.wav"))
    {
        l->background_music_.setVolume(20);
        l->background_music_.play();
        l->background_music_.setLoop(true);
    }

    window.setView(view);
    return l;
}

Level* Level::LoadLevel3(sf::RenderWindow& window, tgui::GuiSFML& gui)
{
    SpriteSpawner::instance()->enable();
    SpriteSpawner::instance()->setDrag(20);
    SpriteSpawner::instance()->setSpawnSpeed(0.4);
    SpriteSpawner::setSpawnRange({-30, -20}, {-50, HEIGTH-50});
    SpriteSpawner::loadNewTexture("res/leaf.png");

    View view(sf::FloatRect(0.f, 0.f, (float) WIDTH, (float) HEIGTH));

    // Textures
    sf::Texture* object_texture = new sf::Texture();
    object_texture->loadFromFile("res/object.png");
    sf::Texture* target_texture = new sf::Texture();
    target_texture->loadFromFile("res/target.png");
    sf::Texture* bg_texture = new sf::Texture();
    bg_texture->loadFromFile("res/autumn.jpg");

    // background
    SpriteObject* background = new SpriteObject(*bg_texture, view.getCenter(), 0);
    background->setScale({0.3f, 0.3f});

    // Splines

    // ParticleDynamics
    ParticleDynamics* pdyn_o = new ParticleDynamics(true);
    Particle* p = new Particle(*object_texture, {WIDTH/2.f, HEIGTH/2.f}, {0, 0}, 10);
    p->sprite_->setScale({1, 1});
    pdyn_o->addParticle(p);
    ForceSource* f_m = new ForceSource(ForceType::Gravity, {0, 0}, 0);
    ForceSource* f_g1 = new ForceSource(ForceType::Constant, {0, 981});
    pdyn_o->addForceSource(f_m);
    pdyn_o->addForceSource(f_g1);
    pdyn_o->addForceSource(p);

    ParticleDynamics* pdyn = new ParticleDynamics(true);
    pdyn->spawner_enabled_ = true;

    ForceSource* f_mouse = new ForceSource(ForceType::AntiGravity, {0, 0}, 800000);
    ForceSource* f_c = new ForceSource(ForceType::Constant, {5000, 0});
    ForceSource* f_g = new ForceSource(ForceType::Constant, {0, 981});

    for (int i = 0; i < 10; i++)
    {   
        sf::Vector2f pos = {std::rand() % WIDTH, std::rand() % HEIGTH};
        ForceSource* f = new ForceSource(ForceType::Gravity, pos, 20000 + std::rand() % 200000);
        pdyn->addForceSource(f);
    }
    pdyn->addForceSource(f_mouse);
    pdyn->addForceSource(f_c);
    pdyn->addForceSource(f_g);

    // RigidBodies

    // Magnetarea
    MagnetArea* ma = new MagnetArea();
    ma->load("res/area_definitions/l3.txt");

    // targetarea
    Spline* ta = new Spline({{150, 120}, {150, HEIGTH/2}, {150, HEIGTH-120}, {WIDTH/2, HEIGTH-100}, 
                            {WIDTH-150, HEIGTH-120}, {WIDTH-150, HEIGTH/2}, {WIDTH-100, 120}, {WIDTH/2, 120}}, *target_texture, true);
    Spline::traversal_speed_ = 20;
    ta->setOrigin(0);
    ta->setScale({0.1f, 0.1f});

    // Walls
    WallArea* wa = new WallArea();
    wa->load("res/area_definitions/l3.txt");

    // Magnets
    MagnetKeySet player1_keyset = { sf::Keyboard::Key::A, sf::Keyboard::Key::D, sf::Keyboard::Key::W, sf::Keyboard::Key::S, sf::Keyboard::Key::E };
    MagnetKeySet player2_keyset = { sf::Keyboard::Key::Left, sf::Keyboard::Key::Right, sf::Keyboard::Key::Up, sf::Keyboard::Key::Down, sf::Keyboard::Key::Enter };

    Magnet* m = new Magnet(player1_keyset, ma->getSpawnPoint(), 0, {0.07f, 0.07f});
    m->setFollowObject(p->sprite_);
    m->setForceSource(f_m);

    // create level
    Level* l = new Level("Level3");
    l->splines_.push_back(ta);
    l->loaded_textures_.push_back(object_texture);
    l->loaded_textures_.push_back(target_texture);
    l->loaded_textures_.push_back(bg_texture);
    l->particle_dynamics_.push_back(pdyn_o);
    l->particle_dynamics_.push_back(pdyn);
    l->game_objects_.push_back(background);
    l->magnets_.push_back(m);
    l->magnet_area_ = ma;
    l->target_area_ = ta;
    l->wall_area_ = wa;
    l->object_ = p->sprite_;

    l->mouse_force = f_mouse;

    if (l->background_music_.openFromFile("res/audio/wind.wav"))
    {
        l->background_music_.setVolume(20);
        l->background_music_.play();
        l->background_music_.setLoop(true);
    }

    window.setView(view);
    return l;
}

Level* Level::LoadLevel4(sf::RenderWindow& window, tgui::GuiSFML& gui)
{
    SpriteSpawner::instance()->enable();
    SpriteSpawner::instance()->setDrag(8);
    SpriteSpawner::instance()->setSpawnSpeed(0.1);
    SpriteSpawner::setSpawnRange({-10, WIDTH+10}, {-10, -20});
    SpriteSpawner::loadNewTexture("res/snowflake.png");

    View view(sf::FloatRect(0.f, 0.f, (float) WIDTH, (float) HEIGTH));

    // Textures
    sf::Texture* object_texture = new sf::Texture();
    object_texture->loadFromFile("res/object.png");
    sf::Texture* bg_texture = new sf::Texture();
    bg_texture->loadFromFile("res/winter.png");

    // background
    SpriteObject* background = new SpriteObject(*bg_texture, view.getCenter(), 0);
    background->setScale({1.42f, 1.42f});

    // Splines

    // ParticleDynamics
    ParticleDynamics* pdyn_o = new ParticleDynamics(true);
    Particle* p = new Particle(*object_texture, {WIDTH/2, HEIGTH/2}, {0, 0}, 10);
    p->sprite_->setScale({1, 1});
    pdyn_o->addParticle(p);
    ForceSource* f_m = new ForceSource(ForceType::Gravity, {0, 0}, 0);
    ForceSource* f_g1 = new ForceSource(ForceType::Constant, {0, 981});
    pdyn_o->addForceSource(f_m);
    pdyn_o->addForceSource(f_g1);
    pdyn_o->addForceSource(p);

    ParticleDynamics* pdyn = new ParticleDynamics(true);
    pdyn->spawner_enabled_ = true;

    ForceSource* f_mouse = new ForceSource(ForceType::AntiGravity, {0, 0}, 800000);
    // ForceSource* f_c = new ForceSource(ForceType::Constant, {5000, 0});
    ForceSource* f_g = new ForceSource(ForceType::Constant, {0, 981});

    for (int i = 0; i < 10; i++)
    {   
        sf::Vector2f pos = {std::rand() % WIDTH, std::rand() % HEIGTH};
        ForceSource* f = new ForceSource(ForceType::Gravity, pos, 30000 + std::rand() % 30000);
        pdyn->addForceSource(f);
    }
    pdyn->addForceSource(f_mouse);
    // pdyn->addForceSource(f_c);
    pdyn->addForceSource(f_g);

    // RigidBodies

    // Magnets

    // Magnetarea

    // targetarea

    // create level
    Level* l = new Level("Level4");
    l->loaded_textures_.push_back(bg_texture);
    l->particle_dynamics_.push_back(pdyn_o);
    l->particle_dynamics_.push_back(pdyn);
    l->game_objects_.push_back(background);

    l->mouse_force = f_mouse;
    l->object_ = p->sprite_;

    if (l->background_music_.openFromFile("res/audio/wind.wav"))
    {
        l->background_music_.setVolume(20);
        l->background_music_.play();
        l->background_music_.setLoop(true);
    }

    window.setView(view);
    return l;
}

Level* Level::LoadLevelParticleDemo(sf::RenderWindow& window, tgui::GuiSFML& gui)
{
    SpriteSpawner::disable();
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
        p->sprite_->setScale({0.5f, 0.5f});
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
    SpriteSpawner::disable();
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

Level* Level::LoadLevel(sf::RenderWindow& window, tgui::GuiSFML& gui, Level* (*levelToLoad)(sf::RenderWindow& window, tgui::GuiSFML& gui))
{
    level_lock.lock();

    Level* c = current_level;
    c->destroy(window);
    gui.remove(c->level_panel_);
    currentLevelFunction = levelToLoad;
    current_level = levelToLoad(window, gui);

    level_lock.unlock();
}