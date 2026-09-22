# predictive_guidance: iLQR-based predictive guidance controller, built on
# ilqr-cpp.

add_library(gnc_predictive_guidance
    src/predictive_guidance_controller.cpp)
add_library(gnc::predictive_guidance ALIAS gnc_predictive_guidance)

target_include_directories(gnc_predictive_guidance PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

target_link_libraries(gnc_predictive_guidance
    PUBLIC gnc::guidance ilqr::ilqr autodiff::autodiff
    PRIVATE gnc_compile_options)

target_compile_features(gnc_predictive_guidance PUBLIC cxx_std_20)

if(GNC_BUILD_TESTS AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests/CMakeLists.txt")
    add_subdirectory(tests)
endif()
