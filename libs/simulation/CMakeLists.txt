# simulation: vehicle models and the fixed-step simulator.

add_library(gnc_simulation
    src/uav_3dof_model.cpp)
add_library(gnc::simulation ALIAS gnc_simulation)

target_include_directories(gnc_simulation PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

target_link_libraries(gnc_simulation
    PUBLIC gnc::math
    PRIVATE gnc_compile_options)

target_compile_features(gnc_simulation PUBLIC cxx_std_20)

if(GNC_BUILD_TESTS AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests/CMakeLists.txt")
    add_subdirectory(tests)
endif()
