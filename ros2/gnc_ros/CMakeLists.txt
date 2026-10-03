cmake_minimum_required(VERSION 3.23)
project(gnc_ros VERSION 0.1.0 LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(gnc_core REQUIRED)
find_package(gnc_interfaces REQUIRED)

add_library(gnc_nodes SHARED
    src/simulation_node.cpp
    src/estimation_node.cpp
    src/guidance_node.cpp)
target_include_directories(gnc_nodes PRIVATE include)
target_compile_features(gnc_nodes PUBLIC cxx_std_20)
set_target_properties(gnc_nodes PROPERTIES CXX_EXTENSIONS OFF)
target_compile_options(gnc_nodes PRIVATE
    $<$<CXX_COMPILER_ID:GNU,Clang,AppleClang>:-Wall;-Wextra;-Wpedantic>)
ament_target_dependencies(gnc_nodes
    rclcpp rclcpp_components rosgraph_msgs gnc_interfaces)
target_link_libraries(gnc_nodes
    gnc::simulation
    gnc::target_trajectory_generator
    gnc::sensor_model
    gnc::target_state_estimation
    gnc::guidance::pn
    gnc::guidance::predictive)

rclcpp_components_register_node(gnc_nodes
    PLUGIN "gnc_ros::SimulationNode" EXECUTABLE simulation_node)
rclcpp_components_register_node(gnc_nodes
    PLUGIN "gnc_ros::EstimationNode" EXECUTABLE estimation_node)
rclcpp_components_register_node(gnc_nodes
    PLUGIN "gnc_ros::GuidanceNode" EXECUTABLE guidance_node)

install(TARGETS gnc_nodes
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin)
install(FILES "${CMAKE_CURRENT_SOURCE_DIR}/../../LICENSE"
    DESTINATION share/${PROJECT_NAME})
ament_package()
