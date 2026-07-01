option(WITH_COPPELIASIM "Build with CoppeliaSim remote API integration" OFF)

set(APP_SOURCES
    src/main.cpp
    src/benchmarking.cpp
)

if(WITH_COPPELIASIM)
    list(APPEND APP_SOURCES
        src/coppeliasimTests.cpp
        src/extApi.c
        src/extApiPlatform.c
    )
endif()

add_executable(ur_app ${APP_SOURCES})
target_include_directories(ur_app PRIVATE src)
target_link_libraries(ur_app PRIVATE ur_kinematics math_lib)

if(WITH_COPPELIASIM)
    set(COPPELIA_REMOTE_API_DIR "${CMAKE_SOURCE_DIR}/cpp/universalRobotsKinematics/Dependencies/CoppeliaSim/remoteApi")
    set(COPPELIA_INCLUDE_DIR    "${CMAKE_SOURCE_DIR}/cpp/universalRobotsKinematics/Dependencies/CoppeliaSim/include")

    target_include_directories(ur_app PRIVATE ${COPPELIA_REMOTE_API_DIR} ${COPPELIA_INCLUDE_DIR})
    target_compile_definitions(ur_app PRIVATE
        WITH_COPPELIASIM
        NON_MATLAB_PARSING
        MAX_EXT_API_CONNECTIONS=255
        DO_NOT_USE_SHARED_MEMORY
    )

    if(WIN32)
        target_link_libraries(ur_app PRIVATE winmm ws2_32)
    elseif(APPLE)
        target_link_libraries(ur_app PRIVATE pthread)
    elseif(UNIX)
        target_link_libraries(ur_app PRIVATE pthread rt)
    endif()
endif()
