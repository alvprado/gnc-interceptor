cmake_minimum_required(VERSION 3.23)
project(gnc_core VERSION 0.1.0 LANGUAGES CXX)

find_package(ament_cmake REQUIRED)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
# Allow static core libraries to link into ROS2 component shared libraries.
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Build type" FORCE)
endif()

option(BUILD_TESTING "Build the unit tests" ON)
set(GNC_BUILD_TESTS "${BUILD_TESTING}")
if(BUILD_TESTING)
    enable_testing()
endif()

add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/../../libs"
                 "${CMAKE_CURRENT_BINARY_DIR}/libs")

include(cmake/GncInstallLibrary.cmake)
gnc_install_library(gnc_math math)
gnc_install_library(gnc_simulation simulation)
gnc_install_library(gnc_target_trajectory_generator target_trajectory_generator)
gnc_install_library(gnc_guidance_common guidance::common)
gnc_install_library(gnc_guidance_pn guidance::pn)
gnc_install_library(gnc_guidance_predictive guidance::predictive)
gnc_install_library(gnc_sensor_model sensor_model)
gnc_install_library(gnc_target_state_estimation target_state_estimation)

# Load the separately exported iLQR target before importing the GNC targets.
ament_export_dependencies(Eigen3 autodiff ilqr)
ament_export_targets(gnc_coreTargets HAS_LIBRARY_TARGET NAMESPACE gnc::)
install(FILES "${CMAKE_CURRENT_SOURCE_DIR}/../../LICENSE"
    DESTINATION share/${PROJECT_NAME})
ament_package()
