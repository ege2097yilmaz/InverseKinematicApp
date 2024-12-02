cmake_minimum_required(VERSION 3.10)
project(URInverseKinematics)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Include directories
include_directories(include)
find_package(Eigen3 REQUIRED)

# Add the executable
add_executable(ur_ik_test src/main.cpp src/utility.cpp src/URInverseKinematics.cpp)

# Link Eigen library
target_link_libraries(ur_ik_test Eigen3::Eigen)
