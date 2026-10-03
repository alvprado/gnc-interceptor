add_library(gnc_simulation_node SHARED
    simulation_node.cpp
    target_configuration.cpp)
target_include_directories(gnc_simulation_node PRIVATE "${PROJECT_SOURCE_DIR}")
ament_target_dependencies(gnc_simulation_node
    rclcpp rcl_interfaces rclcpp_components rosgraph_msgs gnc_interfaces)
target_link_libraries(gnc_simulation_node
    gnc::simulation
    gnc::target_trajectory_generator
    gnc::sensor_model)

rclcpp_components_register_node(gnc_simulation_node
    PLUGIN "gnc_ros::SimulationNode" EXECUTABLE simulation_node)

install(TARGETS gnc_simulation_node
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME}/simulation)

if(BUILD_TESTING)
    ament_add_gtest(test_target_configuration test/test_target_configuration.cpp)
    target_include_directories(test_target_configuration PRIVATE "${PROJECT_SOURCE_DIR}")
    target_link_libraries(test_target_configuration gnc_simulation_node)
endif()
