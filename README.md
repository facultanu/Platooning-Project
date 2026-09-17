# Platooning Project in ROS 2 Jazzy

A multi-robot platooning implementation developed in **ROS 2 Jazzy** using **TurtleBot3 Burger** (compatible with Waffle Pi as well). This project explores advanced cooperative control algorithms, safety guarantees, cyber-security resilience, and formation maneuvers.

---

## Project Overview & Progression

This repository documents an incremental, step-by-step approach to multi-agent robotic control and cooperative driving:

1. **Target & Trajectory Tracking:**
   * Initial practice with ROS 2 publishing and subscribing.
   * Implementation of the saturated control method from the paper *"Cost-effective CLF-based control design for a wheeled robot"* by Trang Huynh Thao Nguyen, Huu-Thinh Do, and Ionela Prodan, alongside Control Lyapunov Function (CLF) tracking principles.

2. **Nominal Platoon Control (CBF):**
   * Implementation of a 3-car platoon leveraging Control Barrier Functions (CBF) to guarantee collision avoidance and safe spacing.
   * Utilizes a path-graph communication topology among members.

3. **Resilient Platoon Control under FDI Attacks:**
   * Extension of the platoon architecture to a full-graph communication topology.
   * Incorporation of attack-resilient mechanisms to handle a False Data Injection (FDI) attack targeting one of the follower robots.

4. **Potential Fields & Maneuvers (APF):**
   * Transitioning away from FDI scenarios to implement Artificial Potential Fields (APF) between formation members for decentralized spacing.
   * Integration of a basic trajectory-based overtaking maneuver.

---

## System Requirements & Dependencies

* **Operating System:** Linux Ubuntu 24.04 (WSL2 works as well)
* **Middleware:** [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/Installation/Alternatives/Ubuntu-Development-Setup.html)
* **Hardware Support:** [TurtleBot3 Drivers](https://docs.robotis.com/docs/systems/turtlebot3/overview/) (configured for `burger`. For `waffle_pi`, make sure to make the necessary modifications)
* **Optimization Libraries:**
  * **OSQP** (Quadratic Programming solver) — *Version 1.1.3*
  * **CVXPY** (Convex optimization & constraint collection) — *Version 1.4.4* *(Note: Newer versions may conflict with NumPy)*

---

## Repository Structure

```text
turtlebot3_ws/
└── src/
    └── platooning_pkg/ 
        ├── launch/         # Simulation and experiment launch scripts
        ├── models/         # turtelbot3 burger model
        ├── platooning_pkg/ # Core Python nodes, controllers (CBF, CBF with FDI APF)
        ├── package.xml     # ROS 2 package dependencies
        └── setup.py        # Python build and entry-point configurations

```
## Installation & Build Guide

### Set up the Workspace Directory
Extract or clone your project files directly into turtlebot3_ws/src source folder:
```bash
cd turtlebot3_ws/src
# Place your platooning_pkg folder here
cd ..
colcon build
source install/local_setup.bash
```

## Launching

### For one robot

```text
source /opt/ros/jazzy/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo empty_world.launch.py
```

### For platoon(s)

```text
ros2 launch platooning_pkg <name_from_launch_folder.launch.py>
```

In another terminal

```text
cd turtlebot3_ws/src
ros2 run platooning_pkg <name_as_it_is_in_setup.py>
```

## Troubleshooting

* **NumPy & CVXPY Version Mismatch:** If optimization solver nodes crash or throw CVXPY/NumPy errors, verify you are explicitly running **CVXPY 1.4.4** and **OSQP 1.1.3**.
* **Missing Package/Node Errors:** If `ros2 run` cannot locate your executable, ensure that you have the actual script in platooning_pkg, then execute `colcon build` and source your environment with `source install/local_setup.bash`.
