add_executable(test_guidance_pn
    test_pn_control_law.cpp
    test_pn_controller.cpp
    test_transverse_control_allocation.cpp)

target_link_libraries(test_guidance_pn
    PRIVATE gnc::guidance::pn
            gnc_compile_options
            GTest::gtest_main)

gtest_discover_tests(test_guidance_pn)
