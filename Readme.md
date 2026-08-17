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