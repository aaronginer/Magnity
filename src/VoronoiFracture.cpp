#include "VoronoiFracture.h"
#include "Level.h"

#include <utility>
#include <iostream>
#include <random>

int id = 0;
extern Level* current_level;

VoronoiFracture::VoronoiFracture(RigidBody* rigidBody, sf::Vector3<double> collisionPoint) {
    this->rigidBody = rigidBody;
    computeVoronoiPoints(collisionPoint);
    //computeDistanceField(rigid_bodies); //using nanoflann

    sf::Image image;
    image.create(rigidBody->body.getSize().x, rigidBody->body.getSize().y, sf::Color::Transparent);
    image = rigidBody->body.getTexture()->copyToImage(); // Set image to the image of the original object
    this->vornoi_image = image;

    sf::Image imgFracture;
    imgFracture.create(image.getSize().x, image.getSize().y, sf::Color::Transparent);

    for(int i = 0; i < this->vPoints.size(); i++) {
        this->rigidBodesImages.push_back(imgFracture);
    }

    //fill colours
    this->colors.push_back(sf::Color::Red);
    this->colors.push_back(sf::Color::Blue);
    this->colors.push_back(sf::Color::Green);
    this->colors.push_back(sf::Color::Cyan);
    this->colors.push_back(sf::Color::Magenta);
    this->colors.push_back(sf::Color::Yellow);
}

void VoronoiFracture::computeVoronoiPoints(sf::Vector3<double> entryPoint) {
    sf::Vector3<double> distObj = rigidBody->x - entryPoint;
    //get quarter that collision lays in - here make more voronoi points
    int concentrated_maxX = 0;
    int concentrated_maxY = 0;
    int concentrated_minX = 0;
    int concentrated_minY = 0;

    //TODO: round number to integer

    if(distObj.x > 0) {
        if(distObj.y > 0) {
            concentrated_maxX = rigidBody->body.getTexture()->getSize().x / 2;
            concentrated_minX = 0.0;
            concentrated_maxY = rigidBody->body.getTexture()->getSize().y;
            concentrated_minY = rigidBody->body.getTexture()->getSize().y / 2;
        }
        else if(distObj.y < 0) {
            concentrated_maxX = rigidBody->body.getTexture()->getSize().x / 2;
            concentrated_minX = 0.0;
            concentrated_maxY = rigidBody->body.getTexture()->getSize().y / 2;
            concentrated_minY = 0.0;
        }
        else { // y is zero
            concentrated_maxX = 0.0;
            concentrated_minX = 0.0;
            concentrated_maxY = rigidBody->body.getTexture()->getSize().y;
            concentrated_minY = 0.0;
        }
    } else if(distObj.x < 0) {
        if(distObj.y > 0) {
            concentrated_maxX = rigidBody->body.getTexture()->getSize().x;
            concentrated_minX = rigidBody->body.getTexture()->getSize().x / 2;;
            concentrated_maxY = rigidBody->body.getTexture()->getSize().y;
            concentrated_minY = rigidBody->body.getTexture()->getSize().y / 2;;
        }
        else if(distObj.y < 0) {
            concentrated_maxX = rigidBody->body.getTexture()->getSize().x;
            concentrated_minX = rigidBody->body.getTexture()->getSize().x / 2;;
            concentrated_maxY = rigidBody->body.getTexture()->getSize().y / 2;;
            concentrated_minY = 0.0;
        }
        else { // y is zero
            concentrated_maxX = rigidBody->body.getTexture()->getSize().x;
            concentrated_minX = rigidBody->body.getTexture()->getSize().x / 2;
            concentrated_maxY = rigidBody->body.getTexture()->getSize().y;
            concentrated_minY = 0.0;
        }
    } else if(distObj.x == 0) {
        if(distObj.y > 0) {
            concentrated_maxX = rigidBody->body.getTexture()->getSize().x;
            concentrated_minX = 0.0;
            concentrated_maxY = rigidBody->body.getTexture()->getSize().y;
            concentrated_minY = rigidBody->body.getTexture()->getSize().y / 2;;
        }
        else if(distObj.y < 0) {
            concentrated_maxX = rigidBody->body.getTexture()->getSize().x;
            concentrated_minX = 0.0;
            concentrated_maxY = rigidBody->body.getTexture()->getSize().y / 2;;
            concentrated_minY = 0.0;
        } else if(distObj.y == 0) {
            concentrated_minX = (rigidBody->body.getTexture()->getSize().x / 2) / 2;
            concentrated_maxX = (rigidBody->body.getTexture()->getSize().x / 2) + concentrated_minX;
            concentrated_minY = (rigidBody->body.getTexture()->getSize().y / 2) / 2;
            concentrated_maxY = (rigidBody->body.getTexture()->getSize().y / 2) + concentrated_minY;
        }
    }

    //https://stackoverflow.com/questions/7560114/random-number-c-in-some-range
    std::random_device rdX; // obtain a random number from hardware
    std::mt19937 genX(rdX()); // seed the generator
    std::uniform_int_distribution<> distrX(concentrated_minX, concentrated_maxX); // define the range

    std::random_device rdY; // obtain a random number from hardware
    std::mt19937 genY(rdY()); // seed the generator
    std::uniform_int_distribution<> distrY(concentrated_minY, concentrated_maxY); // define the range

    //get random voronoi points with a concentration at the collision point
    for(int i = 0; i < 4; i++) {
        float x = distrX(genX);
        float y = distrY(genY);
        //this->voronoiPoints.insert({id, point});
        this->voronoiPoints.push_back({x,y});
        vPoints.push_back({x,y,0});
        vPointsIDX.insert({std::pair<int,int>(x,y), i});
        id++;
    }

    //add two more points
    std::random_device rdGeneral; // obtain a random number from hardware
    std::mt19937 genGeneral(rdGeneral()); // seed the generator
    std::uniform_int_distribution<> distrGeneral(0.0, rigidBody->radius * 2); // define the range

    float x = distrGeneral(genGeneral);
    float y = distrGeneral(genGeneral);
    this->voronoiPoints.push_back({x,y});
    vPoints.push_back({x,y,0});
    vPointsIDX.insert({std::pair<int,int>(x,y), 4});
    id++;

    x = distrGeneral(genGeneral);
    y = distrGeneral(genGeneral);
    vPoints.push_back({x,y,0});
    this->voronoiPoints.push_back({x,y});
    vPointsIDX.insert({std::pair<int,int>(x,y), 5});
}

double VoronoiFracture::isInsideOutside(sf::Vector3<double> vPa, sf::Vector3<double> vPb, sf::Vector3<double> voxel) {
    sf::Vector3<double> sum = vPa + vPb;
    sf::Vector3<double> m = {sum.x / 2, sum.y / 2, sum.z /2};
    sf::Vector3<double> vPb_vPa = vPb - vPa;
    sf::Vector3<double> x = voxel - m;
    double magnitude = std::sqrt((vPb_vPa.x * vPb_vPa.x) + (vPb_vPa.y * vPb_vPa.y));
    double distanceB = std::sqrt(pow(vPb.x - voxel.x, 2) + pow(vPb.y - voxel.y, 2));
    double distanceA = std::sqrt(pow(vPa.x - voxel.x, 2) + pow(vPa.y - voxel.y, 2));

    if(magnitude == 0) {
        vPb_vPa =  {0.0f, 0.0f, 0.0f};
    }
    else {
        vPb_vPa = {(double)(vPb_vPa.x / magnitude), (double)(vPb_vPa.y / magnitude),(double)(vPb_vPa.z / magnitude)};
    }

    if(distanceB > distanceA) {
        return ((x.x * vPb_vPa.x) + (x.y * vPb_vPa.y) + (x.z * vPb_vPa.z)); //pa is nearest
    } else if(distanceB < distanceA) {
        return ((x.x * vPb_vPa.x) + (x.y * vPb_vPa.y) + (x.z * vPb_vPa.z)) * -1; //pa is nearest
    } else { //we have edge
        return 0; //this means we have dge
    }
}

std::pair<sf::Vector3<double>, sf::Vector3<double>> VoronoiFracture::getTwoClosestPoints(sf::Vector3<double> point) {
    sf::Vector3<double> point1;
    sf::Vector3<double> point2;
    double minDist = 10000000000000000.0f;

    for(auto vPoint : vPoints) {
        double distance = std::sqrt(((vPoint.x - point.x) * (vPoint.x - point.x)) + ((vPoint.y - point.y) * (vPoint.y - point.y)));
        if(distance < minDist) {
            minDist = distance;
            point1 = point2;
            point2 = vPoint;
        }
    }

    std::pair<sf::Vector3<double>, sf::Vector3<double>> closestPtns = {point2, point1};
    return closestPtns;
}

int VoronoiFracture::getIndexPtn(sf::Vector3<double> vPoint) {
    int idx = 0;
    for(auto ptn : vPoints) {
        if(ptn.y == vPoint.y && ptn.x == vPoint.x) {
            return idx;
        }
        idx++;
    }

    return -1;
}

void VoronoiFracture::calcualteVoronoiFracture(std::vector<RigidBody*> *insertedBodies) {
    sf::Image img = rigidBody->body.getTexture()->copyToImage();
    for (int i = 0; i < this->rigidBody->body.getTexture()->getSize().x; i++) {
        for (int j = 0; j < this->rigidBody->body.getTexture()->getSize().y; j++) {
            std::pair<sf::Vector3 < double>,
                    sf::Vector3 < double >> closestPoints = getTwoClosestPoints(sf::Vector3<double>(i, j, 0));
            float noise = fbm(sf::Vector3<double>(i, j, 0));
            if(!this->use_noise) {
                noise = 1.0f;
            }
            double d = isInsideOutside(closestPoints.first, closestPoints.second, sf::Vector3<double>(i * noise, j * noise, 0));

            int idxClosestPtn = vPointsIDX.at(std::pair<int, int>(closestPoints.first.x, closestPoints.first.y));
            if (d < 0) {
                // Set the pixel in image_fracture using the corresponding pixel from the array
                sf::Color c = img.getPixel(i, j);
                if(c.a != NULL) {
                    this->rigidBodesImages.at(idxClosestPtn).setPixel(i, j, c);
                }
                if(i == closestPoints.first.x && j == closestPoints.first.y) {
                    this->vornoi_image.setPixel(i, j, sf::Color::Black);
                }
                else {
                    this->vornoi_image.setPixel(i, j, this->colors.at(idxClosestPtn));
                }
            }
        }
    }

    //std::string filename = "img.png"; // Set the desired file name
    //rigidBody->body.getTexture()->copyToImage().saveToFile(filename);

    int i = 0;
    sf::Vector3<double> diff = rigidBody->x - sf::Vector3<double>(10, 10, 0);

    for(auto point : vPoints) {
        sf::Texture* texture = new sf::Texture();
        int idxPtn = vPointsIDX.at(std::pair<int, int>(point.x, point.y));
        texture->loadFromImage(rigidBodesImages.at(idxPtn));
        RigidBody* fracture = new RigidBody(0.001f, 2.5, 0, rigidBody->width, rigidBody->height, *texture, false,
                                            point.x + diff.x, point.y + diff.y, insertedBodies->size());

        //https://stackoverflow.com/questions/7560114/random-number-c-in-some-range
        //std::random_device rdX; // obtain a random number from hardware
        //std::mt19937 genX(rdX()); // seed the generator
        //std::uniform_int_distribution<> distrX(-100, 100); // define the range

        //float rand = distrX(genX);
        fracture->v = rigidBody->v;
        //fracture->v.x -= (double)rand;
        //fracture->v.y -= (double)rand;
        fracture->w = rigidBody->w;
        //fracture->torque_vec = sf::Vector3<double>(0,0, rigidBody->torque_vec.z * 0.0001);
        fracture->torque_vec = rigidBody->torque_vec;
        fracture->splitter = true;
        fracture->nameImg = "img" + std::to_string(idxPtn) + ".png";
        insertedBodies->push_back(fracture);
        i++;
    }

    auto iter = std::find(RigidBody::rigid_bodies->begin(), RigidBody::rigid_bodies->end(), rigidBody);
    assert(iter != RigidBody::rigid_bodies->end());
    for (auto m : current_level->magnets_)
    {
        m->follow_object_ = nullptr;
    }
    auto ptr = *iter;
    delete ptr;
    RigidBody::rigid_bodies->erase(iter);
}

//https://www.shadertoy.com/view/4dS3Wd
float hash(sf::Vector3<double> p) {
    sf::Vector3<double> tmp1 = {p.x * 0.13, p.y * 0.13, 0.0};
    sf::Vector3<double> p3 = {tmp1.x-(long)tmp1.x, tmp1.y-(long)tmp1.y, 0.0};
    sf::Vector3<double> tmp2 = {p3.x + 3.333, p3.y + 3.333, 0.0};
    double dot = (p3.x * tmp2.x) + (p3.y * tmp2.y);
    p3 = {p3.x + dot, p3.y + dot, 0.0};
    float sum = (p3.x + p3.y);
    return sum-(long)sum;
}

//taken from VU - 2D Noise based on Morgan McGuire @morgan3d
//https://www.shadertoy.com/view/4dS3Wd
float VoronoiFracture::noise(sf::Vector3<double> st) {
    sf::Vector3<double> i = {floor(st.x), floor(st.y), 0};
    sf::Vector3<double>  f = {st.x-(long)st.x, st.y-(long)st.y, 0};

    //for corners in 2D of a tile
    float a = hash(i);
    float b = hash(sf::Vector3<double>(i.x + 1.0, i.y + 0.0, 0.0));
    float c = hash(sf::Vector3<double>(i.x + 0.0, i.y + 1.0, 0.0));
    float d= hash(sf::Vector3<double>(i.x + 1.0, i.y + 1.0, 0.0));

    sf::Vector3<double> u = {f.x * f.x, f.y * f.y, 0.0};
    sf::Vector3<double> tmp = {3.0 - (2.0 * f.x), 3.0 - (2.0 * f.y), 0.0};
    u = {u.x * tmp.x, u.y * tmp.y, 0.0};
    float mix = a + (b - a) * u.x;
    return mix + (c - a) * u.y * (1.0 - u.x) + (d - b) * u.x * u.y;
}

#define OCTAVES 6
float VoronoiFracture::fbm (sf::Vector3<double> st) {
    //initial values
    float value = 0.0;
    float amplitude = 0.5;
    //loop for octaves
    for(int i = 0; i < OCTAVES; i++) {
        value += amplitude * noise(st);
        st = {st.x * 2.0, st.y * 2.0, 0.0};
        amplitude *= 0.5;
    }

    return value;
}

void VoronoiFracture::showVornoiCells() {
    sf::Texture* texture = new sf::Texture();
    texture->loadFromImage(this->vornoi_image);
    this->rigidBody->body.setTexture(texture);
}

