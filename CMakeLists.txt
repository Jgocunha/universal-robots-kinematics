cmake_minimum_required(VERSION 3.20)
project(universal_robots_kinematics LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Warnings-as-errors for our own targets only. Applied per-target via
# target_compile_options (never add_compile_options), so FetchContent'd
# dependencies (Eigen, GoogleTest) are unaffected.
add_library(project_warnings INTERFACE)
target_compile_options(project_warnings INTERFACE
    $<$<CXX_COMPILER_ID:MSVC>:/W4 /WX>
    $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-Wall -Wextra -Wpedantic -Werror>
)

# Coverage instrumentation (GCC/Clang --coverage), off by default. Scoped to
# ur_kinematics + its test suite only (never FetchContent'd deps) via this
# INTERFACE target, same pattern as project_warnings above.
option(UR_KINEMATICS_COVERAGE "Enable --coverage instrumentation for ur_kinematics + its test suite" OFF)
add_library(project_coverage INTERFACE)
if(UR_KINEMATICS_COVERAGE)
    target_compile_options(project_coverage INTERFACE --coverage)
    target_link_options(project_coverage INTERFACE --coverage)
endif()

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

# Eigen's own headers trip our -Wall/-Wextra/-Wpedantic /-W4 (its CMakeLists
# doesn't mark its include dirs SYSTEM). Re-add them as SYSTEM on our targets
# so warnings-as-errors only applies to our own code, not template
# instantiations that originate inside Eigen headers.
get_target_property(EIGEN_INCLUDE_DIRS Eigen3::Eigen INTERFACE_INCLUDE_DIRECTORIES)

# --- ur_kinematics: the kinematics library (public headers in include/) ----------
add_library(ur_kinematics STATIC
    src/ur_kinematics.cpp
    src/math_utils.cpp
)
target_include_directories(ur_kinematics
    PUBLIC  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src
)
target_include_directories(ur_kinematics SYSTEM PUBLIC ${EIGEN_INCLUDE_DIRS})
target_link_libraries(ur_kinematics PUBLIC Eigen3::Eigen PRIVATE project_warnings project_coverage)

# --- ur_demo: the console demo application ----------------------------------------
add_executable(ur_demo
    apps/demo/main.cpp
    apps/demo/benchmarking.cpp
)
target_include_directories(ur_demo PRIVATE apps/demo)
target_link_libraries(ur_demo PRIVATE ur_kinematics project_warnings)

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
install(FILES LICENSE README.md DESTINATION .)
