cmake_minimum_required(VERSION 3.23)

project(gnc_interceptor
    VERSION 0.1.0
    DESCRIPTION "Guidance, navigation and control interception stack"
    LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Build type" FORCE)
endif()

option(GNC_BUILD_TESTS "Build the unit tests" ON)

if(GNC_BUILD_TESTS)
    enable_testing()
endif()

add_subdirectory(libs)
add_subdirectory(standalone)
