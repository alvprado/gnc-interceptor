include("${CMAKE_CURRENT_LIST_DIR}/../cmake/GncDependencies.cmake")

add_library(gnc_compile_options INTERFACE)
target_compile_options(gnc_compile_options INTERFACE
    $<$<CXX_COMPILER_ID:GNU,Clang,AppleClang>:-Wall;-Wextra;-Wpedantic;-Wconversion>)

add_subdirectory(math)
add_subdirectory(simulation)
add_subdirectory(target_trajectory_generator)
add_subdirectory(guidance)
add_subdirectory(sensor_model)
add_subdirectory(target_state_estimation)
