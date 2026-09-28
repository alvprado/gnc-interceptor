add_executable(test_target_trajectory_generator
    test_maneuvers.cpp)

target_link_libraries(test_target_trajectory_generator
    PRIVATE gnc::target_trajectory_generator
            gnc::math
            gnc_compile_options
            GTest::gtest_main)

gtest_discover_tests(test_target_trajectory_generator)
