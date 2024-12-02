#include <iostream>
#include "URInverseKinematics.h"
#include <fstream>
#include "matplotlib-cpp/matplotlibcpp.h"

namespace plt = matplotlibcpp;


void plot2D(const std::vector<std::array<double, 3>>& positions, const std::string& plane) {
    std::vector<double> x, y;

    if (plane == "xy") {
        for (const auto& pos : positions) {
            x.push_back(pos[0]);
            y.push_back(pos[1]);
        }
        plt::figure();
        plt::plot(x, y, "-o");
        plt::xlabel("X");
        plt::ylabel("Y");
        plt::title("Robot Arm in X-Y Plane");
        plt::grid(true);
        plt::show();
    } else if (plane == "xz") {
        for (const auto& pos : positions) {
            x.push_back(pos[0]);
            y.push_back(pos[2]);
        }
        plt::figure();
        plt::plot(x, y, "-o");
        plt::xlabel("X");
        plt::ylabel("Z");
        plt::title("Robot Arm in X-Z Plane");
        plt::grid(true);
        plt::show();
    } else {
        std::cerr << "Invalid plane specified. Use 'xy' or 'xz'." << std::endl;
    }
}

void plotRobotStructure(const std::vector<std::array<double, 3>>& positions, const std::string& plane) {
    std::vector<double> x, y;
    std::string xlabel, ylabel, title;

    if (plane == "xy") {
        xlabel = "X";
        ylabel = "Y";
        title = "Robot Arm in X-Y Plane";
        for (const auto& pos : positions) {
            x.push_back(pos[0]);
            y.push_back(pos[1]);
        }
    } else if (plane == "xz") {
        xlabel = "X";
        ylabel = "Z";
        title = "Robot Arm in X-Z Plane";
        for (const auto& pos : positions) {
            x.push_back(pos[0]);
            y.push_back(pos[2]);
        }
    } else {
        std::cerr << "Invalid plane specified. Use 'xy' or 'xz'." << std::endl;
        return;
    }

    // Plot links (lines connecting joints)
    plt::figure();
    plt::plot(x, y, "-o");  // Connect joints with lines

    // Highlight joints with larger markers
    plt::scatter(x, y, 100, {{"color", "red"}});  // Joints as red points

    // Add labels and title
    plt::xlabel(xlabel);
    plt::ylabel(ylabel);
    plt::title(title);
    plt::grid(true);

    // Show the plot
    plt::show();
}

int main() {
    RobotKinematics robot;

    // Add links with DH parameters (example values)
    robot.addLink(0, 0, 1, M_PI / 2);  // Link 1
    robot.addLink(0, 0, 1, 0);        // Link 2
    robot.addLink(0, 0, 1, 0);        // Link 3

    // Target position for IK
    std::array<double, 3> target = { 0.5, -0.3, 0.15};

    std::cout << "Performing inverse kinematics..." << std::endl;
    robot.inverseKinematics(target);

    // std::vector<std::array<double, 3>> positions = { {0, 0, 0} };
    // auto T = robot.forwardKinematics();
    // for (size_t i = 0; i < robot.links.size(); ++i) {
    //     positions.push_back(robot.getEndEffectorPosition());
    // }

    // plot2D(positions, "xy");
    // plot2D(positions, "xz");


    std::vector<std::array<double, 3>> positions = { {0, 0, 0} };  // Base position
    std::array<std::array<double, 4>, 4> T = {{
        {1, 0, 0, 0},
        {0, 1, 0, 0},
        {0, 0, 1, 0},
        {0, 0, 0, 1}
    }};
    for (const auto& link : robot.links) {
        auto T_link = robot.computeTransformationMatrix(link);

        // Multiply T by T_link to get the global transformation
        std::array<std::array<double, 4>, 4> T_next = T;
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                T_next[i][j] = 0;
                for (int k = 0; k < 4; ++k) {
                    T_next[i][j] += T[i][k] * T_link[k][j];
                }
            }
        }

        // Update T and extract joint position
        T = T_next;
        positions.push_back({T[0][3], T[1][3], T[2][3]});
    }

    // Plot in X-Y and X-Z planes
    plotRobotStructure(positions, "xy");
    plotRobotStructure(positions, "xz");

    auto endEffectorPosition = robot.getEndEffectorPosition();
    std::cout << "Final End-Effector Position: (" 
              << endEffectorPosition[0] << ", " 
              << endEffectorPosition[1] << ", " 
              << endEffectorPosition[2] << ")" << std::endl;

    return 0;
}