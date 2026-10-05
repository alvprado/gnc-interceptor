find_package(Eigen3 3.4 REQUIRED NO_MODULE)

add_library(gnc_visualization_node SHARED visualization_node.cpp)
target_include_directories(gnc_visualization_node PRIVATE "${PROJECT_SOURCE_DIR}")
target_link_libraries(gnc_visualization_node Eigen3::Eigen)
ament_target_dependencies(gnc_visualization_node
    rclcpp rcl_interfaces rclcpp_components gnc_interfaces geometry_msgs visualization_msgs)

rclcpp_components_register_node(gnc_visualization_node
    PLUGIN "gnc_ros::VisualizationNode" EXECUTABLE visualization_node)

install(TARGETS gnc_visualization_node
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME}/visualization)

if(BUILD_TESTING)
    ament_add_gtest(test_visualization_node test/test_visualization_node.cpp)
    target_include_directories(test_visualization_node PRIVATE "${PROJECT_SOURCE_DIR}")
    target_link_libraries(test_visualization_node gnc_visualization_node)
endif()
