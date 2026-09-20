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
  swarm_control/          # main ROS 2 package
    include/swarm_control/ # headers (PID, dynamics, consensus)
    src/                   # node implementations
    launch/                # ROS 2 launch files
    worlds/                # Gazebo world files
    models/                # Gazebo SDF drone models
```

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

## Development environment

Developed on Windows via [VS Code Remote - WSL](https://code.visualstudio.com/docs/remote/wsl). The toolchain (ROS 2, Gazebo, compilers) lives in WSL2 Ubuntu 24.04; this repo is accessed at its Windows path (`/mnt/c/...`) from inside WSL. Open this folder in VS Code, then use "Remote-WSL: Reopen Folder in WSL" (Ctrl+Shift+P) to get IntelliSense and terminal access wired to the WSL toolchain.

## License

MIT-based, with a non-military field-of-use restriction. See [LICENSE](LICENSE). Because it restricts who can use the software, this is a **source-available** license, not an OSI-approved "open source" license, even though the code is public and modifiable.
