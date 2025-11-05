# 🚗 Gazebo UGV Autonomous Driving — Waypoint Following with PID Control

> Autonomous waypoint navigation in **Gazebo** using a **PID controller**  
> Developed by **Han Seunghak (한승학)**  

---

## 📘 Overview

This project implements a **PID-based autonomous driving controller** for a UGV in **Gazebo simulation**.  
The vehicle follows predefined **waypoints** until it reaches the **final goal**.  

The system also includes a `pose_tf_broadcaster` node which publishes TF transformations between robot frames —  
this node **must be launched together** with the main controller.

---


---

## 🧩 Node Descriptions

### 🧠 `ugv_controller`
- **Purpose:**  
  Main node responsible for **autonomous navigation** and **PID velocity control**.  
- **Key Functions:**
  - Subscribes to `/odom` for current pose estimation.
  - Reads waypoint list (CSV or hardcoded) and computes target pose.
  - Calculates control errors (distance, heading).
  - Applies **PID control** for linear & angular velocity.
  - Publishes `/cmd_vel` to control the vehicle in Gazebo.
- **Topics:**
  - **Subscribed:** `/odom` (`nav_msgs/Odometry`)
  - **Published:** `/cmd_vel` (`geometry_msgs/Twist`)
- **Parameters:**
  - `Kp_lin`, `Ki_lin`, `Kd_lin` — Linear velocity PID gains  
  - `Kp_ang`, `Ki_ang`, `Kd_ang` — Angular velocity PID gains  
  - `goal_tolerance` — Distance threshold for waypoint switching  
  - `waypoint_file` — Path to CSV file with waypoints  

---

### 🌐 `pose_tf_broadcaster`
- **Purpose:**  
  Publishes TF transformations to maintain correct spatial relationships between frames.  
  Essential for ensuring the controller has consistent position/orientation data.
- **Key Functions:**
  - Subscribes to `/odom` and republishes TF as `map → odom → base_link`.
  - Provides real-time transform broadcasting for RViz and controller alignment.
- **Topics & Frames:**
  - **Subscribed:** `/odom`
  - **Published TF Frames:** `map`, `odom`, `base_link`
- **Launch:**  
  Must always be executed before or together with `ugv_controller`.

---

## ⚙️ Node Connection Overview

```mermaid
graph TD;
    A[Gazebo Simulation] -->|/odom| B[pose_tf_broadcaster];
    B -->|TF (map → base_link)| C[ugv_controller];
    D[Waypoint CSV / Manual Input] -->|Target Waypoints| C;
    C -->|/cmd_vel| A;
