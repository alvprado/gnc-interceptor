add_executable(test_predictive_guidance
    test_predictive_guidance.cpp
    ../src/predictive_guidance_controller.cpp)
target_include_directories(test_predictive_guidance PRIVATE ../include)
target_compile_definitions(test_predictive_guidance PRIVATE EIGEN_INITIALIZE_MATRICES_BY_NAN)
target_link_libraries(test_predictive_guidance PRIVATE
    gnc::guidance::common ilqr::ilqr autodiff::autodiff
    gnc_compile_options GTest::gtest_main)
gtest_discover_tests(test_predictive_guidance)
