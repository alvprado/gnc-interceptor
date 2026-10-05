# ROS2 interception action

Launching the ROS stack displays the initial scene and waits for a goal. Each goal on
`/interception/start` runs one attempt using the configured scenario and guidance controller.
After the attempt finishes, the scene resets and waits for another goal.

Build the workspace and launch it:

```sh
source /opt/ros/jazzy/setup.bash
cd ros2
colcon build --symlink-install
source install/setup.bash
ros2 launch gnc_bringup simulation.launch.py
```

In a second terminal, from the repository root:

```sh
source /opt/ros/jazzy/setup.bash
source ros2/install/setup.bash
ros2 action send_goal /interception/start gnc_interfaces/action/Intercept '{}' --feedback
```

The default interception distance is **1 m**, measured between the ground-truth target and
interceptor positions at the same simulation instant. Set a different threshold in the goal:

```sh
ros2 action send_goal /interception/start gnc_interfaces/action/Intercept \
  '{interception_distance_m: 2.0, max_interception_time_s: 20.0}' --feedback
```

The action rejects nonpositive or nonfinite distances and time limits, and rejects additional
goals while an attempt is active. Feedback reports distance in meters, interceptor altitude in meters, and
elapsed attempt time in seconds, approximately every 0.1 simulation seconds and at termination.

| Event | Action status | Result `success` |
| --- | --- | --- |
| Separation strictly below the requested distance | `SUCCEEDED` | `true` |
| Interceptor altitude strictly below zero | `ABORTED` | `false` |
| Attempt time reaches `max_interception_time_s` | `ABORTED` | `false` |
| Client cancels the goal | `CANCELED` | `false` |
| Manual reset during an attempt | `ABORTED` | `false` |

The result includes a description, final distance, and elapsed attempt time. A ground collision
takes precedence over interception and timeout at the same step; interception takes precedence
over timeout. Initial altitude zero is permitted. There is no automatic retry. Each goal defaults to a 30-second simulated attempt limit after
launch. If the limit is reached, the action aborts with `Interception time limit exceeded.`
A client can
cancel with `rclcpp_action::Client::async_cancel_goal` or `rclpy.action.ActionClient`'s goal
handle `cancel_goal_async()`.

`InterceptionNode` owns the action server. It resets the scene, waits for a valid target
estimate from that reset, and asks `SimulationNode` to start. The interceptor starts at rest
with zero bank, pointing toward the **estimated** target position: heading is
`atan2(dy, dx)` and flight-path angle is `atan2(dz, hypot(dx, dy))`. The dynamics preserve
this attitude at zero speed, so the initial acceleration follows the launch direction.
While moving, both controllers use `n = cos(gamma)` and zero bank during boost to hold
the climb angle; the previous `n = 1` command would keep pitching an inclined launch upward.
The goal aborts if no valid estimate becomes available within five wall-clock seconds.

The action node uses an exact-time synchronizer for the interceptor and target ground-truth
messages. It checks distance and altitude only for matching timestamps and run IDs. Once a
terminal condition is observed, it requests a reset and waits for acknowledgment before
returning the result. Service calls are asynchronous, keeping cancellation responsive.
With separate processes, the simulator can advance a few steps while that request travels;
the result describes the matched sample that triggered completion.

`SimulationNode` owns physics, sensors, and scene state. Its public C++ API is `start()`,
`pause()`, and `reset()`, intended for calls serialized with its callbacks. A thin
`/simulation/control` service exposes these operations to the action node. `START` takes
the estimated target position in the world frame and the expected run ID; it rejects a
stale run, an already running simulation, or an undefined/nonfinite direction. `PAUSE`
holds the scene and discards the command. `RESET` restores the scene and leaves it paused.
These controls can also be used manually, for example:

```sh
ros2 service call /simulation/control gnc_interfaces/srv/SimulationControl '{command: 2}'
```

This replaces the previous `/simulation/reset` service. Normal operation only needs the
action. There is no automatic retry. Control operations have a five-second wall-clock
response timeout; a failed reset is reported in the action result.

On reset, the interceptor returns to rest at the origin, the target trajectory returns to its
initial point, and the previous guidance command is cleared. The `/clock` timestamp keeps
increasing; elapsed trajectory time restarts at zero. Each reset and successful start increments a
`run_id` carried in state, radar, estimate, and command messages. The estimator reinitializes its
EKF for the new run, guidance reconstructs its controller (including boost and predictive
history), and visualization clears its trails. Old-run messages cannot change the new run.
Radar draws fresh noise samples on successive attempts.

The message definitions changed, so rebuild and restart all ROS nodes together. External
guidance publishers must copy the current interceptor state's `run_id` into their commands.
The simulation ignores commands while waiting for a goal or when their run ID does not match.
