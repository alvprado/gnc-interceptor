add_executable(test_target
    test_maneuvers.cpp)

target_link_libraries(test_target
    PRIVATE gnc::target
            gnc::math
            gnc_compile_options
            GTest::gtest_main)

gtest_discover_tests(test_target)
