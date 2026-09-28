add_executable(test_guidance_common
    test_thrust_control_law.cpp)

target_link_libraries(test_guidance_common
    PRIVATE gnc::guidance::common
            gnc_compile_options
            GTest::gtest_main)

gtest_discover_tests(test_guidance_common)
