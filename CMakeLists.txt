cmake_minimum_required(VERSION 3.20)
project(universal_robots_kinematics LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(FetchContent)

FetchContent_Declare(
    Eigen3
    GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
    GIT_TAG        3.4.0
    GIT_SHALLOW    TRUE
)
set(EIGEN_BUILD_DOC OFF CACHE BOOL "" FORCE)
set(BUILD_TESTING   OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(Eigen3)

# --- ur_kinematics: the kinematics library (public headers in include/) ----------
add_library(ur_kinematics STATIC
    src/ur_kinematics.cpp
    src/math_utils.cpp
)
target_include_directories(ur_kinematics
    PUBLIC  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src
)
target_link_libraries(ur_kinematics PUBLIC Eigen3::Eigen)

# --- ur_demo: the console demo application ----------------------------------------
add_executable(ur_demo
    apps/demo/main.cpp
    apps/demo/benchmarking.cpp
)
target_include_directories(ur_demo PRIVATE apps/demo)
target_link_libraries(ur_demo PRIVATE ur_kinematics)

enable_testing()
add_subdirectory(tests)

# --- Install rules (for packaged releases) -----------------------------------------
include(GNUInstallDirs)

install(TARGETS ur_kinematics ur_demo
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
)
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
