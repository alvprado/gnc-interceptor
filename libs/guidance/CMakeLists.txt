# guidance: interceptor control laws (PN now, iLQR later) behind a common
# controller interface.

add_library(gnc_guidance
    src/pn_control_law.cpp
    src/pn_controller.cpp
    src/thrust_control_law.cpp
    src/transverse_control_allocation.cpp)
add_library(gnc::guidance ALIAS gnc_guidance)

target_include_directories(gnc_guidance PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

target_link_libraries(gnc_guidance
    PUBLIC gnc::math
    PRIVATE gnc_compile_options)

target_compile_features(gnc_guidance PUBLIC cxx_std_20)

if(GNC_BUILD_TESTS AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests/CMakeLists.txt")
    add_subdirectory(tests)
endif()
