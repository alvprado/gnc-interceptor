cmake_minimum_required(VERSION 3.23)
project(gnc_ros VERSION 0.1.0 LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(gnc_interfaces REQUIRED)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
# Allow the statically-built core libraries to link into the component shared libraries.
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
add_compile_options(
    "$<$<CXX_COMPILER_ID:GNU,Clang,AppleClang>:-Wall;-Wextra;-Wpedantic>")

option(BUILD_TESTING "Build the ROS adapter tests" ON)
set(GNC_BUILD_TESTS "${BUILD_TESTING}")
if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    enable_testing()
endif()

add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/../../libs" "${CMAKE_CURRENT_BINARY_DIR}/libs")

add_subdirectory(common)
add_subdirectory(simulation)
add_subdirectory(guidance)
add_subdirectory(estimation)
add_subdirectory(visualization)

install(FILES "${CMAKE_CURRENT_SOURCE_DIR}/../../LICENSE"
    DESTINATION share/${PROJECT_NAME})

ament_package()
