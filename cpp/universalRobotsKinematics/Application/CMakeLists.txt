set(APP_SOURCES
    src/main.cpp
    src/benchmarking.cpp
)

add_executable(ur_app ${APP_SOURCES})
target_include_directories(ur_app PRIVATE src)
target_link_libraries(ur_app PRIVATE ur_kinematics math_lib)
