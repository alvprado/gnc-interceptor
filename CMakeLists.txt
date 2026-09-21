cmake_minimum_required(VERSION 3.20)

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

find_package(Eigen3 3.4 REQUIRED NO_MODULE)

include(FetchContent)

# ilqr-cpp: header-only iLQR solver used by libs/predictive_guidance.
FetchContent_Declare(ilqr
    GIT_REPOSITORY https://github.com/alvprado/ilqr-cpp.git
    GIT_TAG d2a73054dbfaec8a536ed5a05239e225970c0bb1
    GIT_SHALLOW FALSE)
FetchContent_MakeAvailable(ilqr)

# Warning flags shared by every target we compile ourselves. Linked PRIVATE by
# each library so the flags never propagate to consumers.
add_library(gnc_compile_options INTERFACE)
target_compile_options(gnc_compile_options INTERFACE
    $<$<CXX_COMPILER_ID:GNU,Clang,AppleClang>:-Wall;-Wextra;-Wpedantic;-Wconversion>)

if(GNC_BUILD_TESTS)
    enable_testing()

    FetchContent_Declare(googletest
        GIT_REPOSITORY https://github.com/google/googletest.git
        GIT_TAG v1.15.2
        GIT_SHALLOW TRUE)
    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(googletest)

    include(GoogleTest)
endif()

add_subdirectory(libs/math)
add_subdirectory(libs/simulation)
add_subdirectory(libs/target)
add_subdirectory(libs/guidance)
add_subdirectory(libs/predictive_guidance)
add_subdirectory(standalone)
