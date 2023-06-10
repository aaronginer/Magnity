#ifndef MAGNITY_VORONOIFRACTURE_H
#define MAGNITY_VORONOIFRACTURE_H


#include "RigidBody.h"
#include <cstdlib>

class VoronoiFracture {
public:
    VoronoiFracture(RigidBody* rigidBody, sf::Vector3<double> collisionPoint);
    void computeVoronoiPoints(sf::Vector3<double> entryPoint);
    double isInsideOutside(sf::Vector3<double> vPa, sf::Vector3<double> vPb, sf::Vector3<double> voxel);
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
    void showVornoiCells();
    float noise(sf::Vector3<double> st);
    float fbm (sf::Vector3<double> st);
    void getVoronoiImage(sf::Image& voronoi_image);
    std::vector<sf::Color> colors;
    static bool use_noise;
    static bool show_cells;
    static void toggleVoronoiView();

private:

};


#endif //MAGNITY_VORONOIFRACTURE_H
