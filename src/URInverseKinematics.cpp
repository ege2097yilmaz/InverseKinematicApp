#include "URInverseKinematics.h"
#include <iostream>
#include <cmath>
#include <stdexcept>


RobotKinematics::RobotKinematics() {}

void RobotKinematics::addLink(double d, double theta, double a, double alpha) {
    links.push_back({d, theta, a, alpha});
}



std::array<std::array<double, 4>, 4> RobotKinematics::forwardKinematics() {
    std::array<std::array<double, 4>, 4> result = {{
        {1, 0, 0, 0},
        {0, 1, 0, 0},
        {0, 0, 1, 0},
        {0, 0, 0, 1}
    }};
    
    for (const auto& link : links) {
        auto T = computeTransformationMatrix(link);
        std::array<std::array<double, 4>, 4> temp = result;

        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                result[i][j] = 0;
                for (int k = 0; k < 4; ++k) {
                    result[i][j] += temp[i][k] * T[k][j];
                }
            }
        }
    }
    return result;
}

std::array<double, 3> RobotKinematics::getEndEffectorPosition() {
    auto T = forwardKinematics();
    return { T[0][3], T[1][3], T[2][3] };  // Extract position
}

std::array<std::array<double, 4>, 4> RobotKinematics::computeTransformationMatrix(const DHParameter& param) {
    double cosTheta = cos(param.theta);
    double sinTheta = sin(param.theta);
    double cosAlpha = cos(param.alpha);
    double sinAlpha = sin(param.alpha);

    return {{
        { cosTheta, -sinTheta * cosAlpha, sinTheta * sinAlpha, param.a * cosTheta },
        { sinTheta, cosTheta * cosAlpha, -cosTheta * sinAlpha, param.a * sinTheta },
        { 0,        sinAlpha,             cosAlpha,             param.d },
        { 0,        0,                    0,                    1 }
    }};
}

std::vector<std::vector<double>> RobotKinematics::computeJacobian() {
    std::vector<std::vector<double>> J(3, std::vector<double>(links.size(), 0.0));
    std::array<std::array<double, 4>, 4> T = {{
        {1, 0, 0, 0},
        {0, 1, 0, 0},
        {0, 0, 1, 0},
        {0, 0, 0, 1}
    }};

    std::array<double, 3> p_end = getEndEffectorPosition();

    for (size_t i = 0; i < links.size(); ++i) {
        auto T_current = computeTransformationMatrix(links[i]);
        std::array<double, 3> z = { T[0][2], T[1][2], T[2][2] };  // Z-axis of current frame
        std::array<double, 3> p = { T[0][3], T[1][3], T[2][3] };  // Position of current frame

        std::array<double, 3> cross = {  // Cross product z x (p_end - p)
            z[1] * (p_end[2] - p[2]) - z[2] * (p_end[1] - p[1]),
            z[2] * (p_end[0] - p[0]) - z[0] * (p_end[2] - p[2]),
            z[0] * (p_end[1] - p[1]) - z[1] * (p_end[0] - p[0])
        };

        for (int j = 0; j < 3; ++j) {
            J[j][i] = cross[j];
        }

        // Update T for next joint
        std::array<std::array<double, 4>, 4> temp = T;
        for (int r = 0; r < 4; ++r) {
            for (int c = 0; c < 4; ++c) {
                T[r][c] = 0;
                for (int k = 0; k < 4; ++k) {
                    T[r][c] += temp[r][k] * T_current[k][c];
                }
            }
        }
    }

    return J;
}

void RobotKinematics::inverseKinematics(const std::array<double, 3>& target, double tolerance, int maxIterations) {
    int iterations = 0;
    while (iterations < maxIterations) {
        std::array<double, 3> current = getEndEffectorPosition();
        std::array<double, 3> error = { target[0] - current[0], target[1] - current[1], target[2] - current[2] };

        double errorMagnitude = std::sqrt(error[0] * error[0] + error[1] * error[1] + error[2] * error[2]);
        std::cout << "error  " << errorMagnitude << std::endl;
        if (errorMagnitude < tolerance) {
            std::cout << "Converged in " << iterations << " iterations!" << std::endl;
            return;
        }

        auto J = computeJacobian();

        // Pseudoinverse computation (basic implementation)
        std::vector<double> deltaTheta(links.size(), 0.0);
        for (size_t i = 0; i < links.size(); ++i) {
            for (size_t j = 0; j < 3; ++j) {
                deltaTheta[i] += J[j][i] * error[j];
            }
        }

        // Update joint angles
        for (size_t i = 0; i < links.size(); ++i) {
            links[i].theta += deltaTheta[i];
        }

        iterations++;
    }

    std::cout << "Failed to converge in " << maxIterations << " iterations!" << std::endl;
}