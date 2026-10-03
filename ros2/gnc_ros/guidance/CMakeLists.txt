add_library(gnc_guidance_node SHARED guidance_node.cpp)
target_include_directories(gnc_guidance_node PRIVATE "${PROJECT_SOURCE_DIR}")
ament_target_dependencies(gnc_guidance_node
    rclcpp rclcpp_components gnc_interfaces)
target_link_libraries(gnc_guidance_node
    gnc::guidance::pn
    gnc::guidance::predictive)

rclcpp_components_register_node(gnc_guidance_node
    PLUGIN "gnc_ros::GuidanceNode" EXECUTABLE guidance_node)

install(TARGETS gnc_guidance_node
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME}/guidance)
