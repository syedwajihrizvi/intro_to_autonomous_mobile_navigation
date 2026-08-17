# Intro to Autonomous Mobile Navigation via Turtlesim

This repository contains Python implementations of mobile robot navigation algorithms using ROS 2 and `turtlesim`. It covers point-to-point tracking via Near-Identity Diffeomorphism (NID) feedback linearization, multi-node tracking sequences, and dynamic obstacle avoidance using Control Lyapunov Functions (CLF) and Control Barrier Functions (CBF).

---

## 📋 Prerequisites

To run these nodes, you need:
* **ROS 2** (Humble, Iron, Jazzy, or later)
* **`turtlesim`** package installed
* **`scipy`** & **`numpy`** (for QP optimization in CBF/CLF exercises)

If you need help installing ROS 2, refer to [this setup tutorial](https://github.com/tchoopojcharoen/ros2_exercise_basic).

---

## 🛠️ Package Configuration & Setup

Once you create your workspace and clone/create the `main_controller` package, update the `package.xml` and `setup.py` files as shown below.

### 1. Dependencies (`package.xml`)

Ensure all required message types and ROS 2 client libraries are included in your `package.xml`:

```xml
  <depend>rclpy</depend>
  <depend>geometry_msgs</depend>
  <depend>std_msgs</depend>
  <depend>turtlesim</depend>
```

### 2. Setting up nodes (`setup.py`) 
Ensure all the neccessary nodes are setup so you can run them
```python
  entry_points={
    'console_scripts': [
        'ex_1_node = main_controller.ex_1_node:main',
        'ex_2_node = main_controller.ex_2_node:main',
        'ex_3_object_node = main_controller.ex_3.object_node:main',
        'ex_3_main_node = main_controller.ex_3.main_node:main',
        'ex_4_cop_node = main_controller.ex_4.cop:main',
        'ex_4_robber_node = main_controller.ex_4.robber:main',
        'ex_5_avoid_obstacle = main_controller.ex_5_node:main',
        'ex_6_avoid_obstacles = main_controller.ex_6_node:main',
    ],
},
```

### 3. Building
Each time you update anything, run the following in sequence from the /ros_ws directory:

```bash
colcon build
source install/setup.bash
```