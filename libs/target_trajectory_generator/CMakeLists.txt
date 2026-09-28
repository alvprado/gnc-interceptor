# target_trajectory_generator: closed-form target trajectories (ground truth).

add_library(gnc_target_trajectory_generator
    src/geometry.cpp
    src/maneuvers.cpp)
add_library(gnc::target_trajectory_generator ALIAS gnc_target_trajectory_generator)

target_include_directories(gnc_target_trajectory_generator PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

target_link_libraries(gnc_target_trajectory_generator
    PUBLIC Eigen3::Eigen
           gnc::math
    PRIVATE gnc_compile_options)

target_compile_features(gnc_target_trajectory_generator PUBLIC cxx_std_20)

if(GNC_BUILD_TESTS AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests/CMakeLists.txt")
    add_subdirectory(tests)
endif()
