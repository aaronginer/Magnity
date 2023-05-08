//
// Created by Laura Pessl on 30.04.23.
//

#include <SFML/Graphics.hpp>
using namespace sf;

#ifndef MAGNITY_ANIMATION_H
#define MAGNITY_ANIMATION_H


class Animation {

    public:
        Animation(Texture* texture, Vector2u imgCount, float switchTime);
        ~Animation();
        void Update(int row, float deltaTime, bool faceRight);

        IntRect uvRect; //this we want to use to display animation

    private:
        Vector2u imageCount;
        Vector2u currentImg;

        float totalTime;
        float switchTime;
};


#endif //MAGNITY_ANIMATION_H
