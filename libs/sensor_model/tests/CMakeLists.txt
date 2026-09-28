add_executable(test_sensor_model
    test_noise_generator.cpp
    test_radar_model.cpp)

target_link_libraries(test_sensor_model
    PRIVATE gnc::sensor_model
            gnc_compile_options
            GTest::gtest_main)

gtest_discover_tests(test_sensor_model)
