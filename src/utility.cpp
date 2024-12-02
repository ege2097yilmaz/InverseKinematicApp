#include "utility.h"

namespace utility {
    std::vector<std::vector<double>> generateTestPoints(double minX, double maxX, double minY, double maxY, double minZ, double maxZ, double step) {
        std::vector<std::vector<double>> points;

        for (double x = minX; x <= maxX; x += step) {
            for (double y = minY; y <= maxY; y += step) {
                for (double z = minZ; z <= maxZ; z += step) {
                    if (std::sqrt(x*x + y*y + z*z) <= (0.8)) { // Limit to maximum reach of 0.8 meters, adjust for your arm
                        points.push_back({x, y, z});
                    }
                }
            }
        }

        return points;
    }

    std::vector<std::vector<double>> generateRandomPoints(int numPoints, double maxRadius) {
        std::vector<std::vector<double>> points;
        std::srand(std::time(0)); // Seed for randomness

        for (int i = 0; i < numPoints; ++i) {
            double r = maxRadius * std::pow((double)std::rand() / RAND_MAX, 1.0 / 3.0); // Uniform sphere sampling
            double theta = 2 * M_PI * ((double)std::rand() / RAND_MAX);
            double phi = acos(2 * ((double)std::rand() / RAND_MAX) - 1);

            double x = r * sin(phi) * cos(theta);
            double y = r * sin(phi) * sin(theta);
            double z = r * cos(phi);

            points.push_back({x, y, z});
        }

        return points;
    }

    std::vector<std::vector<double>> generateSpherePoints(int numPoints, double radius) {
        std::vector<std::vector<double>> points;

        for (int i = 0; i < numPoints; ++i) {
            double theta = 2 * M_PI * ((double)std::rand() / RAND_MAX);
            double phi = acos(2 * ((double)std::rand() / RAND_MAX) - 1);

            double x = radius * sin(phi) * cos(theta);
            double y = radius * sin(phi) * sin(theta);
            double z = radius * cos(phi);

            points.push_back({x, y, z});
        }

        return points;
    }

} // namespace