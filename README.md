# Flockwork

An open-source decentralized drone swarm control system, built from the ground up in C++ on ROS 2 + Gazebo. This project implements the full control stack: a single-agent control through a multi-agent consensus, instead of leaning on a pre-built autopilot (PX4/ArduPilot), so that keeps the control theory visible which is my intent.


# AI

No AI was used in this project throughout, so please do not open an issue or PR if the code is not checked or reviewed, an LLM generating code for this project in itself is fine, but please ensure that it is reviewed.

## Status

Early scaffolding. Following an incremental build order, see [Roadmap](#roadmap).

## Stack

- **ROS 2 Jazzy** (Ubuntu 24.04 / WSL2): inter-agent messaging. Pub/sub over DDS maps naturally onto decentralized neighbor communication.
- **Gazebo Harmonic** (`ros_gz` bridge): rigid-body physics simulation
- **C++17**: controllers, dynamics, consensus logic

## Repo layout

```
src/
  swarm_control/          # main ROS 2 package, the control theory (CT)
    include/swarm_control/ # headers (PID, dynamics, consensus)
    src/                   # node implementations
    launch/                # ROS 2 launch files
    worlds/                # Gazebo world files
    models/                # Gazebo SDF drone models
tests/                    # stress-test scenarios, kept separate from CT
  launch/stress_test.launch.py
  worlds/stress_test.sdf
```

# Obstacle Detection Logic

Two sources feed `ObstacleAvoidance`, both just produce the same `Obstacle{position, radius}` list. The default is a fixed list of known positions or radii passed as launch params. Opt-in with `use_lidar_sensing:=true` (stress test only) adds a simulated 2D lidar per drone: `LidarObstacleDetector` clusters the scan into world-frame obstacle estimates. A real-hardware mounting-offset lookup is sketched but commented out in `consensus_controller_node.cpp`, since the sim assumes the lidar sits at the drone's own origin.(This is work in progress)

## Stress testing

`tests/stress_test.launch.py` (not part of the `swarm_control` package on purpose) exercises stage 3's consensus logic against conditions it isn't designed to handle yet: wind disturbance, static obstacles with no avoidance logic, tight formation spacing, and a real point A to point B transit instead of hovering or orbiting in place.

```bash
ros2 launch tests/launch/stress_test.launch.py
```

This is meant to surface where the current logic breaks down, not to prove it works. A known finding so far: a drone that collides with an obstacle (no avoidance exists yet) combined with sustained wind can drift far off the intended path with nothing pulling it back except the navigation term, a real gap that stage 4 (collision avoidance) is meant to close.

Drone-to-drone spacing now has a real safety floor: a barrier force (`min_safe_distance`, `barrier_gain`) grows sharply as any neighbor gets too close, an unbounded-but-capped tether (`tether_gain`) pulls a drone back once it strays past `desired_spacing` from its nearest neighbor even beyond `interaction_range`, and the final velocity command is rate-limited (`max_cmd_accel`) so it can't jump discontinuously between ticks. `min_safe_distance` defaults to 0.9m, above the X3 model's real ~0.71m rotor-to-rotor collision floor, going below that is a physical collision no control law can prevent. If you see runaway altitude or wild positions during testing, check for stale `ros2 launch` / `parameter_bridge` processes left over from a previous run first (`ps aux | grep gz`), two simulations publishing to the same topic names looks exactly like instability but isn't.

## Roadmap

1. **Single quadrotor control**: rigid-body model in Gazebo, C++ PID node holding a position setpoint. Done, see `single_drone.launch.py`.
2. **N independent drones**: same controller, replicated, no coordination yet(for now). Done, see `swarm.launch.py`.
3. **Decentralized consensus**: drones exchange local state over ROS 2 topics with neighbors and converge on a shared formation/heading. The core "swarm" behavior. Done, see `consensus_controller_node.cpp`, three local forces (separation/cohesion, velocity alignment, acceleration feedforward from a filtered neighbor-state estimate) plus a navigation term toward a shared goal. Verified stable(Note this is in Gazebo) (velocities settle to approx 0, so no divergence or collisions) both standalone and combined with stage 5. Gains tuned, all overridable via launch args, see `desired_spacing`, `interaction_range`, and the `*_gain` arguments on `swarm.launch.py`.
4. **Collision avoidance**: layered on top of consensus (potential fields / velocity obstacles).
5. **Mean-field density control**: a global layer, separate from stage 3's local consensus, that treats the swarm as a collective. Computes a voronoi partition of a target density function from current drone positions (Lloyd's algorithm / Cortes et al. coverage control) and publishes each drone's cell centroid as its target. Individual PID control is unchanged, this only changes what target it chases. Done as well, see `mean_field_controller_node.cpp`. A fully decentralized version (each drone computes its own cell from neighbors only, no central node) is sketched but disabled in `distributed_density_controller_node.cpp`.

## Building

This is a standard [colcon](https://colcon.readthedocs.io/) workspace. From WSL2 (Ubuntu 24.04) with ROS 2 Jazzy sourced:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Running

Single drone (stage 1):

```bash
ros2 launch swarm_control single_drone.launch.py
```

N independent drones (stage 2), grid-spaced so none cross paths, with the mean-field density controller (stage 5) reshaping them toward two target blobs by default:

```bash
ros2 launch swarm_control swarm.launch.py num_drones:=5
```

Add `headless:=true` to run Gazebo server-only, no GUI window (which is useful with no display attached). Add `use_mean_field:=false` to disable the density controller and just get stage 2's independent hovering. Add `use_consensus:=true` to swap in stage 3's flocking control (each drone reacts to its neighbors, not just a fixed setpoint):

```bash
ros2 launch swarm_control swarm.launch.py num_drones:=5 use_consensus:=true
```

Add `orbit_speed:=0.3` (rad/s) to make the mean-field target blobs orbit the center instead of sitting still, so the swarm keeps flying in a moving formation instead of converging once and stopping.

## Development environment

Developed on Windows via [VS Code Remote - WSL](https://code.visualstudio.com/docs/remote/wsl). The toolchain (ROS 2, Gazebo, compilers) lives in WSL2 Ubuntu 24.04; this repo is accessed at its Windows path (`/mnt/c/...`) from inside WSL. Open this folder in VS Code, then use "Remote-WSL: Reopen Folder in WSL" (Ctrl+Shift+P) to get IntelliSense and terminal access wired to the WSL toolchain.

## License

MIT-based, with a non-military field-of-use restriction. See [LICENSE](LICENSE). Because it restricts who can use the software, this is a **source-available** license, not an OSI-approved "open source" license, even though the code is public and modifiable.
