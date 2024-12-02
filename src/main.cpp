#include <iostream>
#include "URInverseKinematics.h"
#include <fstream>



void printMatrix(const std::array<std::array<double, 4>, 4>& matrix) {
    for (const auto& row : matrix) {
        for (const auto& val : row) {
            std::cout << val << " ";
        }
        std::cout << std::endl;
    }
}


int main() {
    RobotKinematics robot;

    // Add links with DH parameters (example values)
    robot.addLink(0, 0, 1, M_PI / 2);  // Link 1
    robot.addLink(0, 0, 1, 0);        // Link 2
    robot.addLink(0, 0, 1, 0);        // Link 3

    // Target position for IK
    std::array<double, 3> target = { 0.5, 1.0, 0.3};

    std::cout << "Performing inverse kinematics..." << std::endl;
    robot.inverseKinematics(target);

    auto endEffectorPosition = robot.getEndEffectorPosition();
    std::cout << "Final End-Effector Position: (" 
              << endEffectorPosition[0] << ", " 
              << endEffectorPosition[1] << ", " 
              << endEffectorPosition[2] << ")" << std::endl;

    return 0;
}