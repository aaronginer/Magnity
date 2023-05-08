//
// Created by Laura Pessl on 30.04.23.
//

#include <SFML/Graphics.hpp>
#include "Animation.h"
using namespace sf;
#ifndef MAGNITY_PLAYER_H
#define MAGNITY_PLAYER_H


class Player {

    public:
        Player(Texture* texture, Vector2u imgCount, float switchTime, float speed);
        ~Player();

        void Update(float deltaTime);
        void Draw(RenderWindow &window);

        Vector2f getPosition();

    private:
        RectangleShape body;
        Animation animation;
        unsigned int row;
        float speed;
        bool faceRight;

};


#endif //MAGNITY_PLAYER_H
