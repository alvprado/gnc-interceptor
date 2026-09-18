add_executable(test_simulation
    test_uav_3dof_model.cpp
    test_simulator.cpp)

target_link_libraries(test_simulation
    PRIVATE gnc::simulation
            gnc_compile_options
            GTest::gtest_main)

gtest_discover_tests(test_simulation)
