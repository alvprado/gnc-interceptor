# standalone: steps the interceptor and a target trajectory together and
# logs both to CSV, for visually verifying the simulation and target stack.

add_executable(standalone
    main.cpp)

target_link_libraries(standalone
    PRIVATE gnc::simulation
            gnc::guidance
            gnc::predictive_guidance
            gnc::target
            gnc::math
            gnc_compile_options)
