add_library(gnc_interception_node SHARED interception_node.cpp)
target_include_directories(gnc_interception_node PRIVATE "${PROJECT_SOURCE_DIR}")
ament_target_dependencies(gnc_interception_node
    rclcpp rclcpp_action rclcpp_components message_filters gnc_interfaces)
target_link_libraries(gnc_interception_node gnc_ros_converters)

rclcpp_components_register_node(gnc_interception_node
    PLUGIN "gnc_ros::InterceptionNode" EXECUTABLE interception_node)
install(TARGETS gnc_interception_node
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin)

if(BUILD_TESTING)
    ament_add_gtest(test_interception_node test/test_interception_node.cpp)
    target_include_directories(test_interception_node PRIVATE "${PROJECT_SOURCE_DIR}")
    target_link_libraries(test_interception_node gnc_interception_node gnc_simulation_node)
endif()
