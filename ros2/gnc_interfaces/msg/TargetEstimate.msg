uint8 VALID=0
uint8 STALE_TIMESTAMP=1
uint8 SINGULAR_INNOVATION_COVARIANCE=2
uint8 UNINITIALIZED=3

gnc_interfaces/TargetState state
# Row-major 9x9 covariance, ordered [px, py, pz, vx, vy, vz, ax, ay, az].
float64[81] covariance
uint8 status 3
