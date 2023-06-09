#include <SFML/Graphics.hpp>
using namespace sf;

#ifndef MAGNITY_BORDER_H
#define MAGNITY_BORDER_H


class Border {
public:
    Border(Texture* texture, const RectangleShape& shape, RenderWindow &window, int pos,  bool fullscreen, int collisionPointDir);
    Border(Texture* texture, int x, int y, float height, float width, int collisionPointDir);
    ~Border();
    void Draw(RenderWindow &window);

    Vector2f getPosition();
    RectangleShape getShape();
    int getCollisionPointDir();

private:
    RectangleShape border;
    bool fullscreen;
    bool xORy;
    int collisionPointDir;
};


#endif //MAGNITY_BORDER_H
