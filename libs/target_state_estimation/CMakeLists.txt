# target_state_estimation: EKF-based target state estimator, fusing noisy
# radar measurements (libs/sensor_model) into a target position/velocity/
# acceleration estimate.

add_library(gnc_target_state_estimation
    src/extended_kalman_filter.cpp)
add_library(gnc::target_state_estimation ALIAS gnc_target_state_estimation)

target_include_directories(gnc_target_state_estimation PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

target_link_libraries(gnc_target_state_estimation
    PUBLIC gnc::math gnc::sensor_model
    PRIVATE gnc_compile_options)

target_compile_features(gnc_target_state_estimation PUBLIC cxx_std_20)

if(GNC_BUILD_TESTS AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests/CMakeLists.txt")
    add_subdirectory(tests)
endif()
