//
// Created by Laura Pessl on 30.04.23.
//

#include "Animation.h"

Animation::Animation(sf::Texture *texture, sf::Vector2u imgCount, float switchTime)
{
    this->imageCount = imgCount;
    this->switchTime = switchTime;
    totalTime = 0.0f;
    currentImg.x = 0;

    uvRect.width = texture->getSize().x / float(imgCount.x);
    uvRect.height = texture->getSize().y / float(imgCount.y);
}

Animation::~Animation() {

}

void Animation::Update(int row, float deltaTime, bool faceRight)
{
    currentImg.y = row;
    totalTime += deltaTime;

    if(totalTime >= switchTime)
    {
        totalTime -= switchTime;
        currentImg.x++;

        if(currentImg.x >= imageCount.x) {
            currentImg.x = 0; //start first pic again
        }
    }

    uvRect.top = currentImg.y * uvRect.height;

    if(faceRight) {
        uvRect.left = currentImg.x * uvRect.width;
        uvRect.width = abs(uvRect.width);
    }
    else {
        uvRect.left = (currentImg.x + 1) * abs(uvRect.width);
        uvRect.width = -abs(uvRect.width);
    }
}