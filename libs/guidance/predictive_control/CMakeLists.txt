# guidance/predictive_control: iLQR-based predictive guidance controller,
# built on ilqr-cpp, behind the common controller interface
# (gnc::guidance::common).

add_library(gnc_guidance_predictive
    src/predictive_guidance_controller.cpp)
add_library(gnc::guidance::predictive ALIAS gnc_guidance_predictive)

target_include_directories(gnc_guidance_predictive PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

target_link_libraries(gnc_guidance_predictive
    PUBLIC gnc::guidance::common ilqr::ilqr autodiff::autodiff
    PRIVATE $<BUILD_INTERFACE:gnc_compile_options>)

target_compile_features(gnc_guidance_predictive PUBLIC cxx_std_20)

if(GNC_BUILD_TESTS AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests/CMakeLists.txt")
    add_subdirectory(tests)
endif()
