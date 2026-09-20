add_executable(test_guidance
    test_pn_control_law.cpp
    test_pn_controller.cpp
    test_thrust_control_law.cpp
    test_transverse_control_allocation.cpp)

target_link_libraries(test_guidance
    PRIVATE gnc::guidance
            gnc_compile_options
            GTest::gtest_main)

gtest_discover_tests(test_guidance)
