#include <SFML/Graphics.hpp>
#include "Animation.h"
using namespace sf;

#ifndef MAGNITY_OBJECT_H
#define MAGNITY_OBJECT_H


class Object {
    public:
        Object(Texture* texture);
        ~Object();

        void Update(float deltaTime);
        void Draw(RenderWindow &window);

        Vector2f getPosition();

    private:
        RectangleShape body;
};


#endif //MAGNITY_OBJECT_H
