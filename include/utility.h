#include <vector>
#include <cmath>
#include <cstdlib> 
#include <ctime>

namespace utillity{
    std::vector<std::vector<double>> generateTestPoints(double minX, double maxX, double minY, double maxY, double minZ, double maxZ, double step);

    std::vector<std::vector<double>> generateRandomPoints(int numPoints, double maxRadius) ;

    std::vector<std::vector<double>> generateSpherePoints(int numPoints, double radius) ;

}