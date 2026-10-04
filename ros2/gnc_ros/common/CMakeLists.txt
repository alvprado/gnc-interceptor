add_library(gnc_ros_converters SHARED converters.cpp)
target_include_directories(gnc_ros_converters PUBLIC "${PROJECT_SOURCE_DIR}")
ament_target_dependencies(gnc_ros_converters builtin_interfaces gnc_interfaces)
target_link_libraries(gnc_ros_converters gnc::simulation gnc::sensor_model)

install(TARGETS gnc_ros_converters
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin)

if(BUILD_TESTING)
    ament_add_gtest(test_converters test/test_converters.cpp)
    target_link_libraries(test_converters gnc_ros_converters)
endif()
