add_library(gnc_estimation_node SHARED
    estimation_node.cpp
    parameters/ekf_config.cpp
    parameters/estimation_config.cpp)
target_include_directories(gnc_estimation_node PRIVATE "${PROJECT_SOURCE_DIR}")
ament_target_dependencies(gnc_estimation_node
    rclcpp rcl_interfaces rclcpp_components gnc_interfaces)
target_link_libraries(gnc_estimation_node
    gnc_ros_converters
    gnc::target_state_estimation)

rclcpp_components_register_node(gnc_estimation_node
    PLUGIN "gnc_ros::EstimationNode" EXECUTABLE estimation_node)

install(TARGETS gnc_estimation_node
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME}/estimation)

if(BUILD_TESTING)
    ament_add_gtest(test_ekf_config test/test_ekf_config.cpp)
    target_include_directories(test_ekf_config PRIVATE "${PROJECT_SOURCE_DIR}")
    target_link_libraries(test_ekf_config gnc_estimation_node)

    ament_add_gtest(test_estimation_node test/test_estimation_node.cpp)
    target_include_directories(test_estimation_node PRIVATE "${PROJECT_SOURCE_DIR}")
    target_link_libraries(test_estimation_node gnc_estimation_node)
endif()
