add_library(gnc_simulation_node SHARED
    simulation_node.cpp
    interceptor_config.cpp
    sensor_config.cpp
    target_config.cpp)
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
    ament_add_gtest(test_target_config test/test_target_config.cpp)
    target_include_directories(test_target_config PRIVATE "${PROJECT_SOURCE_DIR}")
    target_link_libraries(test_target_config gnc_simulation_node)

    ament_add_gtest(test_interceptor_config test/test_interceptor_config.cpp)
    target_include_directories(test_interceptor_config PRIVATE "${PROJECT_SOURCE_DIR}")
    target_link_libraries(test_interceptor_config gnc_simulation_node)

    ament_add_gtest(test_sensor_config test/test_sensor_config.cpp)
    target_include_directories(test_sensor_config PRIVATE "${PROJECT_SOURCE_DIR}")
    target_link_libraries(test_sensor_config gnc_simulation_node)
endif()
