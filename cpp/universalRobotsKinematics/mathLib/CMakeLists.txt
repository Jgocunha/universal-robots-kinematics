add_library(math_lib STATIC src/mathLib.cpp)
target_include_directories(math_lib PUBLIC src)
target_link_libraries(math_lib PUBLIC Eigen3::Eigen)
