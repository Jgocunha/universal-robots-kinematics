add_library(ur_kinematics STATIC src/universalRobotsKinematics.cpp)
target_include_directories(ur_kinematics PUBLIC src)
target_link_libraries(ur_kinematics PUBLIC math_lib Eigen3::Eigen)
