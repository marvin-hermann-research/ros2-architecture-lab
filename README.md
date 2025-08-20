# Bipedal Robot Behaviour Tree Framework for ROS2

## Project Description

This project provides a **modular research framework** for robotic locomotion in ROS2.  
It combines **Behavior Trees (BTs)** with a **distributed sensor–actuator architecture**, enabling structured decision-making and extensible control of humanoid robot systems.

## Motivation

ROS2 has become the standard middleware for robotics and embodied AI research, offering robust communication between sensors, actuators, and higher-level logic.
Behavior Trees extend this by introducing **structured, hierarchical decision-making**, which is more flexible and transparent than finite-state machines, especially in locomotion tasks.

The framework was designed to explore **scientific conventions in modular robotic architectures**, emphasizing reproducibility, extensibility, and clarity.  
It serves as a foundation for simulated experiments while maintaining compatibility with real hardware through a plug-and-play design.

## Project Context

Context: This project was designed with the afterthought of creating a fully fledeged realistic ros2 robot locomotion framework which not only is inline with todays convention but also modular and expandable for future use of my private research projects

Ziel: As this is my first project my goal was not to haggle with real sensors and actuators but to simulate and build a framwork around this limitation which supports real hardware down the line per plug and play

references: While studying ros2 I mainly used the official documantation of [ROS, ROS2, Py_trees, Py_trees_ros as well as python-json-logger]. During the projcets creation I also documented everything I've learned in a structured and tutoring manner in my ROS2 Master post which you also can find on my git hub [ROS2 Masterpost] I highly reccommend checking it out as this document provides more inside behind the architecture and possibilities of ROS2

## Architecture Overview

```bash
bipedal_robot_pkg/
├── setup.py
├── package.xml
├── requirements.txt
├── __init__.py
├── behaviour_tree/
│   ├── patterns/
│   │   ├── all_patterns.yaml
│   │   ├── idle_pattern.yaml
│   │   └── walk_forward_pattern.yaml
│   ├── nodes/
│   │   ├── tree_factory.py
│   │   ├── actions/
│   │   │   ├── idle_behaviour.py
│   │   │   └── walk_forward_behaviour.py
│   │   └── conditions/
│   │       ├── can_walk.py
│   │       └── must_walk.py
│   └── launch/
│       └── bipedal_robot_application.py
├── ros_nodes/
│   ├── actuators/
│   │   ├── left_leg_node.py
│   │   └── right_leg_node.py
│   ├── controller/
│   │   └── movement_controller_node.py
│   ├── evaluators/
│   │   ├── can_walk_evaluator.py
│   │   └── must_walk_evaluator.py
│   ├── logger/
│   │   └── logging_factory.py
│   ├── sensors/
│   │   ├── battery_monitor_node.py
│   │   ├── imu_sensor_node.py
│   │   └── laser_sensor_node.py
│   └── action_publishers/
│       ├── idle_publisher.py
│       └── walk_forward_publisher.py
```

Besides the standard ROS2 components setup.py, package.xml and the requirements.txt, the project consists of two mein modules:

### behavour_tree/
This module implements all behaviour tree components, stores the locomotion patterns and handels the robots initialization. It contains the following modules:
##### patterns/
Module which contains all locomotion patterns as yaml files 
##### nodes/
Module which contains the behavour tree factory responsible for building the desired tree as well as all the trees action and condition nodes
##### launch/
Module which contains the robot systems launch file which initializates not only the behaviour tee but also all ros2 components

### ros_nodes/
This module implements all ros2 components, handels the systems scientific logging and bridges to the behavour tree. it contains the following modules:
##### actuators/
Module which contains the robots intended actuator nodes e.g. pair of legs. These get their instructions through topics send by the controler module 
##### controller/
Module which contains the systems movement controler. This structure is respobsible for implementing the trees current locomotion action by sending the specific yaml movement information to the actuators
##### evaluators/
Module which bridges the gap from the ros2 components to the behavour tree conditions by imlementing evaluators. This structures listen to specific ros2 topics and are registered to distinct blackboard keys whos data they manipulate
##### loggers/
Module which implements the logging factory which provides the service of a scientific json logger intended to be used for indepth analysis of hypothetical experiments
##### sensors/
Module which contains all the projects sensor nodes. Each node publishes (simmulated) data on distinct topics.
##### action_publishers/
Moudle which bridges the gap from the behaviour tree to the ros2 components by implementing action publishers. These structures are publisher nodes parsed to the trees action nodes on instantiiation. The trees action nodes use them to send instructions to the controler module

[todo paket / klassen diagramme] 

### Genaue erklärung der kritischen komponenten

(wie genau funktionirt das launchen und initialisieren des systems und was passiert dabei (evtl ablauf diagramm))
(wie genau wird der baum angelegt)


## Ablauf projekt start und allgemeiner datenfluss

(wie kommuniziert genau tree mit ros2 teil und was kommuniziert mit wem und was macht der logger)

1. Sensor-Nodes → liefern Daten
2. Evaluatoren → setzen Flags auf die Blackboard
3. BT entscheidet (walk / idle)
4. Action Publisher + Controller → Bewegungen
5. Logger → überwacht alles

Gifs der folgenden dinge:
- projekt start (colcon build, source, ros2 launch)
- rqt graph
- topic list
- node list
- sensor topic streams
- BT anzeigen lassen
- grafik des loggers

## 2 anwendungsfälle 

nennen also idle und laufen. hier fluss diagramme

- gif des anwendungsfalls command eingeben um laufen zu signalisieren, hier zeige haupt ros konsole, konsole des baums, baum, command eingabe console

## Installation & Usage

### Requirements

- ROS2 Humble (or compatible)
- Python ≥ 3.10
- Dependencies from `requirements.txt`

### Setup

#### 1. System requirements

This project is built for **ROS 2 Humble (Ubuntu 22.04)**.  
Make sure ROS 2 Humble is installed following the [official installation guide](https://docs.ros.org/en/humble/Installation.html).  
Also install the colcon build tools:

```bash
sudo apt install python3-colcon-common-extensions
```

#### 2. Clone repository

```bash
git clone https://github.com/marvin-hermann-research/ros2-architecture-lab.git
cd bipedal_robot_pkg
```
#### 3. Install Python dependencies

```bash
pip install -r requirements.txt
```

#### Build the workspace

```bash
colcon build
source install/setup.bash
```

#### Run the application

```bash
ros2 run bipedal_robot_pkg robot_application
```

## References

- [ROS2 Documentation](https://docs.ros.org/)
- [py_trees](https://py-trees.readthedocs.io/)
- [py_trees_ros](https://py-trees-ros-tutorials.readthedocs.io/)
- [python-json-logger](https://pypi.org/project/python-json-logger/)
  
## Extensibility

The framework is designed for modular expansion:
- Integration of additional sensors or actuators.
- New locomotion patterns (AI-generated).
- Alternative or more complex BT structures (e.g. task switching).
- Extension of the logging system (e.g. Grafana dashboards, external monitoring).
- Planned future work: **hardware integration** via a dedicated companion [repository].

## License
This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.

## Author
Marvin Hermann (@marvin-hermann-research)