# guidance/pn_control: PN guidance controller behind the common controller
# interface (gnc::guidance::common).

add_library(gnc_guidance_pn
    src/pn_control_law.cpp
    src/pn_controller.cpp
    src/transverse_control_allocation.cpp)
add_library(gnc::guidance::pn ALIAS gnc_guidance_pn)

target_include_directories(gnc_guidance_pn PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

target_link_libraries(gnc_guidance_pn
    PUBLIC gnc::guidance::common
    PRIVATE $<BUILD_INTERFACE:gnc_compile_options>)

target_compile_features(gnc_guidance_pn PUBLIC cxx_std_20)

if(GNC_BUILD_TESTS AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests/CMakeLists.txt")
    add_subdirectory(tests)
endif()
