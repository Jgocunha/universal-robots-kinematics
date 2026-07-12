# tests/CMakeLists.txt — golden characterization suite (task 00).
# Mirrors the Eigen FetchContent pattern used by the library: pinned release tag,
# shallow clone. The library itself is untouched by this directory.

include(FetchContent)

FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG        v1.15.2
    GIT_SHALLOW    TRUE
)
# On Windows/MSVC, link GoogleTest against the shared CRT to match the default.
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
# We only need GoogleTest to build the test suite, not ship it in release archives
# (cmake --install would otherwise pull in its headers/libs/CMake package config).
set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

include(GoogleTest)

# Best-effort capture of the generating commit for the golden JSON header.
set(GOLDEN_GIT_COMMIT "unknown")
find_package(Git QUIET)
if(GIT_FOUND)
    execute_process(
        COMMAND ${GIT_EXECUTABLE} rev-parse HEAD
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        OUTPUT_VARIABLE GOLDEN_GIT_COMMIT
        OUTPUT_STRIP_TRAILING_WHITESPACE
        ERROR_QUIET
    )
endif()

set(GOLDEN_DIR "${CMAKE_CURRENT_SOURCE_DIR}/golden")

# --- Golden data generator (run once; NOT registered with CTest) ------------------
add_executable(golden_generator golden_generator.cpp)
target_include_directories(golden_generator PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(golden_generator PRIVATE ur_kinematics project_warnings)
target_compile_definitions(golden_generator PRIVATE
    GOLDEN_DIR="${GOLDEN_DIR}"
    GIT_COMMIT="${GOLDEN_GIT_COMMIT}"
)

# --- Characterization test suite --------------------------------------------------
add_executable(ur_kinematics_tests
    test_sanity.cpp
    test_golden_fk.cpp
    test_golden_ik.cpp
    test_validation.cpp
    test_math_utils.cpp
    test_robot_parameters.cpp
    test_pose_and_fk_anchors.cpp
    test_properties.cpp
)
target_include_directories(ur_kinematics_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
# test_math_utils.cpp unit-tests calcTransformationMatrix directly, which is a
# private library implementation detail (src/math_utils.h), not part of the public
# include/ API.
target_include_directories(ur_kinematics_tests PRIVATE ${CMAKE_SOURCE_DIR}/src)
target_link_libraries(ur_kinematics_tests PRIVATE ur_kinematics GTest::gtest_main project_warnings project_coverage)
target_compile_definitions(ur_kinematics_tests PRIVATE
    GOLDEN_DIR="${GOLDEN_DIR}"
)

gtest_discover_tests(ur_kinematics_tests)
