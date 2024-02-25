cmake_minimum_required(VERSION 3.15)

project(universal-robots-kinematics
        DESCRIPTION "A C++ library to compute Universal Robots forward and inverse kinematics."
        LANGUAGES CXX
)

# Define library version
set(DNF_COMPOSER_VERSION_MAJOR 1 CACHE STRING "major version" FORCE)
set(UR_KIN_VERSION_MINOR 0 CACHE STRING "minor version" FORCE)
set(UR_KIN_VERSION ${UR_KIN_VERSION_MAJOR}.${UR_KIN_VERSION_MINOR} CACHE STRING "version" FORCE)

set(UR_KIN_LIB_PREFIX "release")
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    set(UR_KIN_LIB_PREFIX "debug")
endif()

set(UR_KIN_INC_INSTALL_DIR "${CMAKE_INSTALL_PREFIX}/include")
set(UR_KIN_RUNTIME_INSTALL_DIR "${CMAKE_INSTALL_PREFIX}/bin")
set(UR_KIN_LIBRARY_INSTALL_DIR "${CMAKE_INSTALL_PREFIX}/lib/${UR_KIN_LIB_PREFIX}")
set(UR_KIN_ARCHIVE_INSTALL_DIR "${CMAKE_INSTALL_PREFIX}/lib/${UR_KIN_LIB_PREFIX}")
set(UR_KIN_FRAMEWORK_INSTALL_DIR "${CMAKE_INSTALL_PREFIX}/bin")
set(UR_KIN_CMAKE_CONFIG_INSTALL_DIR "${CMAKE_INSTALL_PREFIX}/share/${CMAKE_PROJECT_NAME}")
set(UR_KIN_ADDITIONAL_FILES_INSTALL_DIR "${CMAKE_INSTALL_PREFIX}/share/${CMAKE_PROJECT_NAME}")

# Set C++ standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Check whether VCPKG is set up in your system
if(NOT DEFINED ENV{VCPKG_ROOT})
    message(FATAL_ERROR "ERROR: This project requires VCPKG.\n")
endif()

# Set VCPKG root directory
set(VCPKG_ROOT $ENV{VCPKG_ROOT})

# Include VCPKG toolchain
include(${VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake)

# Set header files
set(header
	"include/ur.h" 
	"include/ur_parameters.h" 
    "include/my_math.h"
)

# Set source files
set(src 
   "src/ur.cpp"
   "src/ur_parameters.cpp"
   "src/my_math.cpp"
)

# Define library target
add_library(${CMAKE_PROJECT_NAME} 
    ${header}
    ${src}
)
target_include_directories(${CMAKE_PROJECT_NAME} PUBLIC 
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}> 
    $<INSTALL_INTERFACE:${UR_KIN_INC_INSTALL_DIR}> 
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Setup dependencies
find_package(Eigen3 CONFIG REQUIRED)
target_link_libraries(${CMAKE_PROJECT_NAME} PRIVATE Eigen3::Eigen)

target_compile_definitions(${CMAKE_PROJECT_NAME} PUBLIC
    DNF_COMPOSER=1
    UR_KIN_VERSION_MAJOR=${UR_KIN_VERSION_MAJOR}
    UR_KIN_VERSION_MINOR=${UR_KIN_VERSION_MINOR}
)

set_target_properties(${CMAKE_PROJECT_NAME} PROPERTIES
    OUTPUT_NAME "${CMAKE_PROJECT_NAME}-${UR_KIN_VERSION}"
    POSITION_INDEPENDENT_CODE ON
)

install(TARGETS ${CMAKE_PROJECT_NAME} EXPORT ${CMAKE_PROJECT_NAME}Targets
    RUNTIME       DESTINATION ${UR_KIN_RUNTIME_INSTALL_DIR}
    LIBRARY       DESTINATION ${UR_KIN_LIBRARY_INSTALL_DIR}
    ARCHIVE       DESTINATION ${UR_KIN_ARCHIVE_INSTALL_DIR}
    FRAMEWORK     DESTINATION ${UR_KIN_FRAMEWORK_INSTALL_DIR}
)

include(CMakePackageConfigHelpers)
set(version_file "${CMAKE_CURRENT_BINARY_DIR}/cmake/${CMAKE_PROJECT_NAME}-config-version.cmake")
write_basic_package_version_file(${version_file}
    VERSION ${UR_KIN_VERSION}
    COMPATIBILITY AnyNewerVersion
)

# Install configuration-specific headers (debug/release)
install(FILES ${header} DESTINATION ${UR_KIN_INC_INSTALL_DIR})

install(EXPORT ${CMAKE_PROJECT_NAME}Targets DESTINATION ${UR_KIN_CMAKE_CONFIG_INSTALL_DIR}
    FILE ${CMAKE_PROJECT_NAME}-config.cmake
)

# Add executable project
set(EXE_PROJECT ${CMAKE_PROJECT_NAME}-exe)
add_executable(${EXE_PROJECT} "main.cpp")
target_include_directories(${EXE_PROJECT} PRIVATE include)
target_link_libraries(${EXE_PROJECT} PRIVATE ${CMAKE_PROJECT_NAME} Eigen3::Eigen)


# Setup Catch2
enable_testing()
find_package(Catch2 CONFIG REQUIRED)
include(CTest)
include(Catch)

# Add test project
set(TEST_PROJECT ${CMAKE_PROJECT_NAME}-test)
add_executable(${TEST_PROJECT} 
    tests/test.cpp
	tests/test_ur.cpp
	tests/test_ur_parameters.cpp
	tests/test_math.cpp
)
target_include_directories(${TEST_PROJECT} PRIVATE include)
target_link_libraries(${TEST_PROJECT} PRIVATE Catch2::Catch2 Catch2::Catch2WithMain ${CMAKE_PROJECT_NAME})
# Automatically discover and add tests
catch_discover_tests(${TEST_PROJECT})