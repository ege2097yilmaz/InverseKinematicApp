#ifndef UR_INVERSE_KINEMATICS_H
#define UR_INVERSE_KINEMATICS_H


#include <vector>
#include <cmath>
#include <array>
#include <iostream>
#include "utility.h"


struct DHParameter {
    double d;       // Link offset
    double theta;   // Joint angle
    double a;       // Link length
    double alpha;   // Twist angle
};


class RobotKinematics {
public:
    RobotKinematics();
    std::vector<DHParameter> links;
    
    void addLink(double d, double theta, double a, double alpha);
    std::array<std::array<double, 4>, 4> computeTransformationMatrix(const DHParameter& param);
    std::array<std::array<double, 4>, 4> forwardKinematics();
    std::array<double, 3> getEndEffectorPosition();
    void inverseKinematics(const std::array<double, 3>& target, double tolerance = 1e-2, int maxIterations = 100000);

private:
    
    std::vector<std::vector<double>> computeJacobian();
    void printMatrix(const std::vector<std::vector<double>>& matrix);
};

#endif // UR_INVERSE_KINEMATICS_H