# Reactive Locomotion Behavior Tree for ROS-Based Humanoid Simulation

## Project Description

This repository implements a modular behavior tree architecture for controlling the reactive lower-body locomotion of a simulated humanoid robot in ROS2. The system combines sensory inputs with decision logic defined in a hierarchical behavior tree using the `py_trees` framework.

## Research Context & Motivation

Behavior Trees (BTs) offer a flexible and scalable alternative to classical Finite State Machines in robotics. In this project, BTs are used to orchestrate real-time reactive locomotion based on sensor-driven feedback from joint states, inertial measurements, and simulated terrain conditions.

This work is inspired by recent publications in cognitive robotics and modular control systems, particularly focusing on real-time reactivity and failure recovery in dynamic environments.

## System Architecture

The system consists of two main layers:
- **ROS2 Node Layer:** Provides data acquisition (e.g., `/right_leg/joint_states`) and actuator control.
- **Behavior Tree Layer (py_trees):** Encodes high-level reactive behaviors such as walking, stopping, balancing, and emergency recovery.

![System Architecture Diagram] #TODO

## Features

- Sensor-driven locomotion control
- Modular and extensible behavior tree structure
- Separation of actuation and decision logic via ROS interfaces

## Installation

```bash
# Clone the repository into your ROS2 workspace
cd ~/ros2_ws/src
git clone https://github.com/marvin-hermann-research/ros2-architecture-lab.git

# Install dependencies
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y

# Build
colcon build --packages-select bipedal_robot_pkg
source install/setup.bash
```

## Running the Siystem

```bash
# Launch simulation
ros2 launch humanoid_simulation bringup.launch.py

# Launch the behavior tree controller
ros2 launch reactive_locomotion_bt locomotion_bt.launch.py
```

## File Structure

```bash
bipedal_robot_pkg/
├── __init__.py
├── behaviour_tree/
│   ├── launch/
│   │   └── simulation_launch.py
│   ├── nodes/
│   │   ├── tree_factory.py
│   │   ├── actions/
│   │   │   ├── idle_behaviour.py
│   │   │   └── walk_forward_behaviour.py
│   │   └── conditions/
│   │       ├── can_walk.py
│   │       └── must_walk.py
│   └── patterns/
│       ├── all_patterns.yaml
│       └── walk_forward_pattern.yaml
├── ros_nodes/
│   ├── actuators/
│   │   ├── left_leg_node.py
│   │   └── right_leg_node.py
│   ├── controller/
│   │   ├── behaviour_command_interface.py
│   │   └── movement_controller_node.py
│   ├── evaluators/
│   │   └── can_walk_evaluator.py
│   ├── logger/
│   │   └── status_logger_node.py
│   └── sensors/
│       ├── battery_monitor_node.py
│       ├── imu_sensor_node.py
│       └── laser_sensor_node.py
```
