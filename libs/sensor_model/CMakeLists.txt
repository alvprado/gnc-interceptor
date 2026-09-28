# sensor_model: radar measurement model producing a noisy [range, range_rate,
# azimuth, elevation] measurement from target/interceptor truth states.

add_library(gnc_sensor_model
    src/radar_model.cpp)
add_library(gnc::sensor_model ALIAS gnc_sensor_model)

target_include_directories(gnc_sensor_model PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

target_link_libraries(gnc_sensor_model
    PUBLIC gnc::math
    PRIVATE gnc_compile_options)

target_compile_features(gnc_sensor_model PUBLIC cxx_std_20)

if(GNC_BUILD_TESTS AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests/CMakeLists.txt")
    add_subdirectory(tests)
endif()
