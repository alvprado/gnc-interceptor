# standalone: steps the interceptor and a target trajectory together and
# logs both to CSV, for visually verifying the simulation and target stack.

add_executable(standalone
    main.cpp)

target_link_libraries(standalone
    PRIVATE gnc::simulation
            gnc::guidance::pn
            gnc::guidance::predictive
            gnc::sensor_model
            gnc::target_trajectory_generator
            gnc::math
            gnc_compile_options)
