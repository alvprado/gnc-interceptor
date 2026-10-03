add_library(gnc_estimation_node SHARED estimation_node.cpp)
target_include_directories(gnc_estimation_node PRIVATE "${PROJECT_SOURCE_DIR}")
ament_target_dependencies(gnc_estimation_node
    rclcpp rclcpp_components gnc_interfaces)
target_link_libraries(gnc_estimation_node gnc::target_state_estimation)

rclcpp_components_register_node(gnc_estimation_node
    PLUGIN "gnc_ros::EstimationNode" EXECUTABLE estimation_node)

install(TARGETS gnc_estimation_node
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME}/estimation)
