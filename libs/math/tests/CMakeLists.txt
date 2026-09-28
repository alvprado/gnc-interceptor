add_executable(test_math
    test_angles.cpp)

target_link_libraries(test_math
    PRIVATE gnc::math
            gnc_compile_options
            GTest::gtest_main)

gtest_discover_tests(test_math)
