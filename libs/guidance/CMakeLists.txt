# guidance: common utilities, a PN guidance controller and a predictive
# (iLQR) guidance controller, each behind a common controller interface
# (see common/include/guidance/controller_concept.hpp).

add_subdirectory(common)
add_subdirectory(pn_control)
add_subdirectory(predictive_control)
