#ifndef MAGNITY_VORONOIFRACTURE_H
#define MAGNITY_VORONOIFRACTURE_H


#include "RigidBody.h"
#include <cstdlib>
#include <nanoflann.hpp>
#include "KDTreeVectorOfVectorsAdaptor.h"

class VoronoiFracture {
public:
    VoronoiFracture(RigidBody* rigidBody, sf::Vector3<double> collisionPoint);
    void computeVoronoiPoints(sf::Vector3<double> entryPoint);
    void computeDistanceField(std::vector<RigidBody*> *rigid_bodies);
    double isInsideOutside(sf::Vector3<double> vPa, sf::Vector3<double> vPb, sf::Vector3<double> voxel);
    std::vector<RigidBody*> getInsertedBodiesVec();
    std::vector<std::vector<double>> voronoiPoints;
    std::vector<sf::Vector3<double>> vPoints;
    std::map<std::pair<int, int>, size_t> distanceField;
    std::map<std::pair<int, int>, int> vPointsIDX;
    std::map<int, std::vector<std::pair<int, int>>> distanceFieldOrdered;
    RigidBody* rigidBody;
    std::vector<RigidBody*> insertedBodies;
    std::pair<sf::Vector3<double>, sf::Vector3<double>> getTwoClosestPoints(sf::Vector3<double> point);
    int getIndexPtn(sf::Vector3<double> vPoint);
    std::vector<sf::Image> rigidBodesImages;
    std::vector<sf::Texture> textures;
    void calcualteVoronoiFracture(std::vector<RigidBody*> *insertedBodies);
    float noise(sf::Vector3<double> st);
    float fbm (sf::Vector3<double> st);

private:

};


#endif //MAGNITY_VORONOIFRACTURE_H
