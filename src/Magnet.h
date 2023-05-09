#include <SFML/Graphics.hpp>
#include "PlayerArea.h"
using namespace sf;

#ifndef MAGNITY_MAGNET_H
#define MAGNITY_MAGNET_H


class Magnet {
    public:
        Magnet(Texture* texture, View& view, int player);
        ~Magnet();

        void Draw(RenderWindow &window);
        Vector2f getPosition();

        RectangleShape& getMagnet();

    private:
        RectangleShape magnet;
        int player;
};


#endif //MAGNITY_MAGNET_H
