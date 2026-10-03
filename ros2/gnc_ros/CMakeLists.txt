cmake_minimum_required(VERSION 3.23)
project(gnc_ros VERSION 0.1.0 LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(gnc_core REQUIRED)
find_package(gnc_interfaces REQUIRED)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
add_compile_options(
    "$<$<CXX_COMPILER_ID:GNU,Clang,AppleClang>:-Wall;-Wextra;-Wpedantic>")

option(BUILD_TESTING "Build the ROS adapter tests" ON)
if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
endif()

add_subdirectory(simulation)
add_subdirectory(guidance)
add_subdirectory(estimation)

install(FILES "${CMAKE_CURRENT_SOURCE_DIR}/../../LICENSE"
    DESTINATION share/${PROJECT_NAME})

ament_package()
