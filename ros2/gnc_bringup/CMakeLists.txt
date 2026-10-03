cmake_minimum_required(VERSION 3.23)
project(gnc_bringup VERSION 0.1.0 LANGUAGES NONE)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config rviz
    DESTINATION share/${PROJECT_NAME})
install(FILES "${CMAKE_CURRENT_SOURCE_DIR}/../../LICENSE"
    DESTINATION share/${PROJECT_NAME})

option(BUILD_TESTING "Build the launch tests" ON)
if(BUILD_TESTING)
    find_package(launch_testing_ament_cmake REQUIRED)
    add_launch_test(test/test_simulation_launch.py TIMEOUT 60)
endif()

ament_package()
