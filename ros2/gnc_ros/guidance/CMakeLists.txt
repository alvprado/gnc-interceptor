add_library(gnc_guidance_node SHARED
    guidance_node.cpp
    parameters/controller_config.cpp
    parameters/guidance_config.cpp)
target_include_directories(gnc_guidance_node PRIVATE "${PROJECT_SOURCE_DIR}")
ament_target_dependencies(gnc_guidance_node
    rclcpp rcl_interfaces rclcpp_components gnc_interfaces)
target_link_libraries(gnc_guidance_node
    gnc_ros_converters
    gnc::guidance::pn
    gnc::guidance::predictive)

rclcpp_components_register_node(gnc_guidance_node
    PLUGIN "gnc_ros::GuidanceNode" EXECUTABLE guidance_node)

install(TARGETS gnc_guidance_node
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME}/guidance)

if(BUILD_TESTING)
    ament_add_gtest(test_controller_config test/test_controller_config.cpp)
    target_include_directories(test_controller_config PRIVATE "${PROJECT_SOURCE_DIR}")
    target_link_libraries(test_controller_config gnc_guidance_node)

    ament_add_gtest(test_guidance_node test/test_guidance_node.cpp)
    target_include_directories(test_guidance_node PRIVATE "${PROJECT_SOURCE_DIR}")
    target_link_libraries(test_guidance_node gnc_guidance_node)
endif()
