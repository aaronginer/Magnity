#include "Level.h"

Level::Level()
{
}

void Level::destroy()
{
    for (Spline* s : this->splines_)
    {
        delete s;
    }

    for (Particle* p : this->particle_dynamics_->particles)
    {
        delete p;
    }
    for (ForceSource* f : this->particle_dynamics_->force_sources)
    {
        delete f;
    }

    delete this->particle_dynamics_;

    /*for (RigidBody* r : this->rigid_bodies_)
    {
        delete r;
    }*/

    for (sf::Texture* t : loaded_textures_)
    {
        delete t;
    }

    //delete magnet1;
    //delete magnet2;
}

void Level::update(float time_delta)
{
    for (Spline* s : this->splines_)
    {
        s->update(time_delta);
    }

    this->particle_dynamics_->update(time_delta);

    /*for (RigidBody* r : this->rigid_bodies_)
    {
        // update rb
    }*/
}

void Level::updateMouseParticlePosition(sf::Vector2f new_pos)
{
    if (mouse_force == nullptr) return;
    mouse_force->x = new_pos;
}

void Level::draw(sf::RenderWindow& renderWindow, float delta_time)
{
    for (Spline* s : this->splines_)
    {
        s->drawCurve();
        s->drawArcSamples();
        s->drawControlPoints();
        s->drawObject();
    }

    this->particle_dynamics_->draw(renderWindow, delta_time);
    this->particle_dynamics_->drawForceField(renderWindow);
    this->particle_dynamics_->drawTrail(renderWindow);

    /*for (RigidBody* r : this->rigid_bodies_)
    {
        // draw rb
    }*/

    magnet1->updateRotation();
    magnet1->draw(renderWindow);
    magnet2->updateRotation();
    magnet2->draw(renderWindow);
}

void Level::handlePolledKeyInput(sf::Event keyEvent)
{
    magnet1->handlePolledKeyInput(keyEvent);
    magnet2->handlePolledKeyInput(keyEvent);
}

void Level::handleInstantKeyInput(float delta_time)
{
    magnet1->handleInstantKeyInput(delta_time);
    magnet2->handleInstantKeyInput(delta_time);
}

Level* Level::LoadLevel1(sf::View& view)
{
    // Textures
    sf::Texture* objectTexture = new sf::Texture();
    objectTexture->loadFromFile("res/object.png");

    // Splines

    Spline* s = new Spline({{500, 200}, {200, 200}, {200, 500}, {500, 500}}, *objectTexture, true);
    
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

    // create level
    Level* l = new Level();
    l->splines_.push_back(s);
    l->particle_dynamics_ = pdyn;
    l->loaded_textures_.push_back(objectTexture);
    l->mouse_force = f;

    // Magnets
    MagnetKeySet player1_keyset = { sf::Keyboard::Key::A, sf::Keyboard::Key::D, sf::Keyboard::Key::W, sf::Keyboard::Key::S, sf::Keyboard::Key::E };
    MagnetKeySet player2_keyset = { sf::Keyboard::Key::Left, sf::Keyboard::Key::Right, sf::Keyboard::Key::Up, sf::Keyboard::Key::Down, sf::Keyboard::Key::Enter };

    l->magnet1 = new Magnet(player1_keyset, {40, 30}, 0);
    l->magnet1->setFollowObject(p1->sprite_);
    l->magnet2 = new Magnet(player2_keyset, {40, 500}, 1);
    l->magnet2->setFollowObject(p1->sprite_);
    return l;
}