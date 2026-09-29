add_executable(test_target_state_estimation
    test_extended_kalman_filter.cpp)

target_link_libraries(test_target_state_estimation
    PRIVATE gnc::target_state_estimation
            gnc_compile_options
            GTest::gtest_main)

gtest_discover_tests(test_target_state_estimation)
