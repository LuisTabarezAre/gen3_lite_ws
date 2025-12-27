# gen3_lite_ws

![License](https://img.shields.io/badge/license-Apache--2.0-blue)

## 🚀 Overview

This workspace contains the *Gen3 Lite* robotic arm simulation for **ROS 2 (Jazzy)** with **Gazebo Harmonic** integration, using `ros2_control` for joint management and control.

Supported features:

- URDF/Xacro model of the Gen3 Lite robotic arm
- Simulation with **Gazebo / ROS 2**
- Multi-robot support (namespaced robots with independent controllers)
- Controllers for **position**, **velocity**, and **effort**
- Works with `gz_ros2_control` plugin
- Example multi-robot launch files (Python and XML)

---

## 📁 Repository Structure
<pre> <code> 
src/gen3_lite_description
├── CMakeLists.txt
├── config
│   ├── gazebo_bridge.yaml
│   ├── gen3_lite_controllers.yaml
│   ├── joint_limits.yaml
│   ├── multi_gen3_lite_controllers.yaml
│   └── twist_limits.yaml
├── launch
│   ├── display_2.launch.py
│   ├── display.launch.py
│   ├── display.launch.xml
│   ├── multi_robot.launch.py
│   ├── multi_robot.launch.xml
│   └── single_robot_ns.launch.py
├── meshes
│   ├── arm
│   │   ├── arm_link.STL
│   │   ├── base_link.STL
│   │   ├── forearm_link.STL
│   │   ├── lower_wrist_link.STL
│   │   ├── shoulder_link.STL
│   │   └── upper_wrist_link.STL
│   └── gripper
│       ├── gripper_base_link.STL
│       ├── left_finger_dist_link.STL
│       ├── left_finger_prox_link.STL
│       ├── right_finger_dist_link.STL
│       └── right_finger_prox_link.STL
├── package.xml
├── rviz
│   └── rviz_config.rviz
├── urdf
│   ├── gen3_lite_macro.xacro
│   └── gen3_lite.urdf.xacro
└── worlds
    ├── multi_table.sdf
    └── table.sdf
</code> </pre>
---

## 📦 Dependencies

Make sure you have installed:

- ROS 2 **Jazzy** or later
- `ros2_control`
- `ros_gz_sim`
- `ros_gz_bridge`
- `controller_manager`
- `joint_state_broadcaster`
- `position_controllers`, `velocity_controllers`, `effort_controllers`

Install dependencies with:

```bash
rosdep update
rosdep install --from-paths src --ignore-src -y

