# target: closed-form target trajectories (ground truth).

add_library(gnc_target
    src/geometry.cpp
    src/maneuvers.cpp)
add_library(gnc::target ALIAS gnc_target)

target_include_directories(gnc_target PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

target_link_libraries(gnc_target
    PUBLIC Eigen3::Eigen
    PRIVATE gnc::math
            gnc_compile_options)

target_compile_features(gnc_target PUBLIC cxx_std_20)

if(GNC_BUILD_TESTS AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests/CMakeLists.txt")
    add_subdirectory(tests)
endif()
