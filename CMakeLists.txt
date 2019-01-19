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
add_library(
        libtrail
        src/trail/Robot.cpp src/trail/Robot.h
        src/trail/Spline.cpp src/trail/Spline.h
        src/trail/utils.cpp src/trail/utils.h
        src/trail/Waypoint.cpp src/trail/Waypoint.h
        src/trail/RobotTrajectory.cpp src/trail/RobotTrajectory.h
        src/outputs/IOutput.h src/outputs/DesmosOutput.h
)
target_link_libraries(libtrail Eigen3::Eigen)

# Unit testing w/googletest
add_executable(runTests test/main.cpp)
target_link_libraries(runTests gtest gtest_main Eigen3::Eigen libtrail)
add_test(runTests runTests)

# Main script
add_executable(trail src/trail-cli.cpp)
target_link_libraries(trail libtrail)