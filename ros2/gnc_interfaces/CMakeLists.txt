cmake_minimum_required(VERSION 3.23)
project(gnc_interfaces VERSION 0.1.0 LANGUAGES C CXX)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
    "msg/InterceptorState.msg"
    "msg/TargetState.msg"
    "msg/RadarMeasurement.msg"
    "msg/TargetEstimate.msg"
    "msg/GuidanceCommand.msg"
    "srv/ResetSimulation.srv"
    DEPENDENCIES std_msgs geometry_msgs)

ament_export_dependencies(rosidl_default_runtime)
install(FILES "${CMAKE_CURRENT_SOURCE_DIR}/../../LICENSE"
    DESTINATION share/${PROJECT_NAME})
ament_package()
