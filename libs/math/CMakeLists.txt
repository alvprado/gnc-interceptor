# math: header-only numerics (concepts, integration policies).

add_library(gnc_math INTERFACE)
add_library(gnc::math ALIAS gnc_math)

target_include_directories(gnc_math INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

target_link_libraries(gnc_math INTERFACE Eigen3::Eigen)

target_compile_features(gnc_math INTERFACE cxx_std_20)

if(GNC_BUILD_TESTS AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests/CMakeLists.txt")
    add_subdirectory(tests)
endif()
