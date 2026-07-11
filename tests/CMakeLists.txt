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
target_link_libraries(golden_generator PRIVATE ur_kinematics)
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
)
target_include_directories(ur_kinematics_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(ur_kinematics_tests PRIVATE ur_kinematics GTest::gtest_main)
target_compile_definitions(ur_kinematics_tests PRIVATE
    GOLDEN_DIR="${GOLDEN_DIR}"
)

gtest_discover_tests(ur_kinematics_tests)
