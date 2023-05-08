#include <SFML/Graphics.hpp>
using namespace sf;

#ifndef MAGNITY_PLAYERAREA_H
#define MAGNITY_PLAYERAREA_H


class PlayerArea {

    public:
        PlayerArea(Texture* texture, RenderWindow &window, int player);
        ~PlayerArea();

        void Draw(RenderWindow &window);

        Vector2f getPosition();

        RectangleShape getArea();


    private:
        RectangleShape area;
};


#endif //MAGNITY_PLAYERAREA_H
