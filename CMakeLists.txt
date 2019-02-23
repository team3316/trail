cmake_minimum_required(VERSION 3.10) # VS 2017 has CMake 3.12
project(trail)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/lib/cmake)

# Eigen
set(EIGEN3_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/lib/eigen3)
find_package(Eigen3 3.3 REQUIRED)

# Googletest
if (APPLE)
    add_definitions(-DGTEST_USE_OWN_TR1_TUPLE)
    add_definitions(-D__GLIBCXX__)
endif (APPLE)
ADD_SUBDIRECTORY(lib/googletest)
enable_testing()

# nlohmann/json
set(JSON_BuildTests OFF CACHE INTERNAL "")
ADD_SUBDIRECTORY(lib/modern-json)

# Trail
add_library(
        libtrail
        src/trail/Robot.cpp src/trail/Robot.h
        src/trail/Spline.cpp src/trail/Spline.h
        src/trail/utils.cpp src/trail/utils.h
        src/trail/Waypoint.cpp src/trail/Waypoint.h
        src/trail/RobotTrajectory.cpp src/trail/RobotTrajectory.h
        src/trail/RobotOrigins.cpp src/trail/RobotOrigins.h
        src/trail/MotionProfile.cpp src/trail/MotionProfile.h
        src/outputs/IOutput.h src/outputs/DesmosOutput.h src/outputs/CSVOutput.h
)
target_link_libraries(libtrail Eigen3::Eigen nlohmann_json::nlohmann_json)

# Unit testing w/googletest
add_executable(runTests test/main.cpp)
target_link_libraries(runTests gtest gtest_main Eigen3::Eigen libtrail)
add_test(runTests runTests)

# Main script
add_executable(trail src/trail-cli.cpp)
target_link_libraries(trail libtrail)
