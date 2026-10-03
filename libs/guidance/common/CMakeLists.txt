# guidance/common: the controller concept, vehicle model parameters and the
# trim thrust law shared by every guidance controller.

add_library(gnc_guidance_common
    src/thrust_control_law.cpp)
add_library(gnc::guidance::common ALIAS gnc_guidance_common)

target_include_directories(gnc_guidance_common PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

target_link_libraries(gnc_guidance_common
    PUBLIC gnc::math
    PRIVATE $<BUILD_INTERFACE:gnc_compile_options>)

target_compile_features(gnc_guidance_common PUBLIC cxx_std_20)

if(GNC_BUILD_TESTS AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests/CMakeLists.txt")
    add_subdirectory(tests)
endif()
