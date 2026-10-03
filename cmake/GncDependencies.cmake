find_package(Eigen3 3.4 REQUIRED NO_MODULE)
find_package(autodiff REQUIRED CONFIG)

include(FetchContent)

# Keep the solver revision identical for the standalone and ROS2 builds.
FetchContent_Declare(ilqr
    GIT_REPOSITORY https://github.com/alvprado/ilqr-cpp.git
    GIT_TAG d2a73054dbfaec8a536ed5a05239e225970c0bb1
    GIT_SHALLOW FALSE)
FetchContent_MakeAvailable(ilqr)

if(GNC_BUILD_TESTS)
    find_package(GTest QUIET CONFIG)
    if(NOT GTest_FOUND)
        FetchContent_Declare(googletest
            GIT_REPOSITORY https://github.com/google/googletest.git
            GIT_TAG v1.15.2
            GIT_SHALLOW TRUE)
        set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
        option(INSTALL_GTEST "Install GoogleTest" OFF)
        FetchContent_MakeAvailable(googletest)
    endif()
    include(GoogleTest)
endif()
