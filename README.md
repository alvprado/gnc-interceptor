# GNC Interceptor

A modern C++ project for exploring guidance, navigation, and control (GNC) in a simulated 3D interception scenario. A point-mass interceptor follows a moving target using proportional navigation (PN) or predictive guidance based on iterative Linear Quadratic Regulator (iLQR) optimization.

The code separates vehicle simulation, target trajectories, sensor modeling, state estimation, and guidance into reusable libraries. 

## Architecture

![GNC architecture: target trajectory through sensor modeling, EKF estimation, and guidance to the interceptor simulator, with interceptor state feedback.](docs/media/gnc_architecture_diagram.svg)

## Components

| Component | Implementation |
| --- | --- |
| Simulation | 3DOF point-mass dynamics actuated with thrust, load factor and bank commands considering state and control limits. The standalone example uses RK4 integration. |
| Target Trajectory Generator | Deterministic constant-velocity, circular, figure-eight, and helical trajectories, including position, velocity, and acceleration. |
| PN guidance | Proportional Navigation (PN) based acceleration commands, control allocation into load factor and bank angle, and a separate boost/trim thrust law. |
| Predictive guidance | Receding-horizon iLQR optimization of load factor and bank angle, with bounded controls and an adaptive horizon. Thrust is handled separately using the same boost/trim approach as the PN controller. |
| Sensor model | Simulates a body-fixed radar seeker sensor providing range, range-rate, azimuth, and elevation measurements with Gaussian noise. |
| Target estimation | Nine-state EKF for position, velocity, and acceleration, using a constant-acceleration model with white-noise jerk. |

The simulator's internal state is `[x, y, z, speed, heading, flight_path_angle]`; its control input is `[thrust, load_factor, bank_angle]`. The inertial frame is **right-handed with z pointing up**.

The predictive controller (iLQR-based MPC) uses a similar model to the simulation but assumes constant speed (hence no speed dynamics) to be symmetric to the classic PN guidance stack. The model is augmented with the previous control input to be able to penalize control change instead of control magnitude.

Out-of-scope for this project: full rigid-body 6DOF dynamics and low-level attitude control are outside its scope. Aerodynamic effects are neglected, only a simple drag model is used in simulation. The interceptor state is assumed to be perfectly known. Sensor model is unbiased.

## Build

Prerequisites:

- A C++20 compiler and a native build tool such as Make or Ninja.
- **CMake 3.23 or newer**: the root project declares 3.20, but the pinned iLQR dependency requires 3.23.
- **Eigen 3.4 or newer**, discoverable as `Eigen3` by CMake.
- **ilqr-cpp**, the iLQR library, auto-fetched by CMake.
- **autodiff**, installed with its CMake package and `autodiff::autodiff` target. Predictive guidance requires it; it is not fetched automatically.

From the repository root:

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build build --target standalone 
```


## Run the simulation

Run from the repository root:

```sh
mkdir -p standalone/outputs
./build/standalone/standalone
```

## Plot results

The plotting script requires Python 3.9 or newer, NumPy, pandas, and Matplotlib:

```sh
python3 -m pip install -r standalone/visualization/requirements.txt
python3 standalone/visualization/plot_trajectory.py standalone/outputs/standalone_trajectory.csv
```

## Repository layout

```text
docs/media/                       Architecture diagram
libs/
  math/                           Shared states, angles, concepts, integrators
  simulation/                     Vehicle dynamics and templated simulator
  target_trajectory_generator/     Analytic target maneuvers
  guidance/
    common/                       Controller concept and thrust control
    pn_control/                   PN and transverse control allocation
    predictive_control/           iLQR controller, dynamics, and cost terms
  sensor_model/                   Radar measurements and Gaussian noise
  target_state_estimation/         Extended Kalman Filter
standalone/
  main.cpp                        Simulation entry point and scenario settings
  csv_logger.hpp                  Run logging
  visualization/                  Python analysis tools
  outputs/                        Generated CSV and plots (ignored by Git)
```
