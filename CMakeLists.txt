cmake_minimum_required(VERSION 3.20)
project(universal_robots_kinematics LANGUAGES CXX C)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_subdirectory(cpp/universalRobotsKinematics)

enable_testing()
add_subdirectory(tests)
