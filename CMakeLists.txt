cmake_minimum_required(VERSION 3.13)
project(trail)

set(CMAKE_CXX_STANDARD 14)

# Eigen
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# Googletest
if (APPLE)
    add_definitions(-DGTEST_USE_OWN_TR1_TUPLE)
    add_definitions(-D__GLIBCXX__)
endif (APPLE)
ADD_SUBDIRECTORY(lib/gtest)
enable_testing()

# Trail
add_library(trail src/Robot.cpp src/Robot.h src/Spline.cpp src/Spline.h src/utils.cpp src/utils.h src/Waypoint.cpp src/Waypoint.h src/RobotTrajectory.cpp src/RobotTrajectory.h)
target_link_libraries(trail Eigen3::Eigen)

# Unit testing w/googletest
add_executable(runTests test/main.cpp)
target_link_libraries(runTests gtest gtest_main Eigen3::Eigen trail)
add_test(runTests runTests)