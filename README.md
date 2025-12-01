UR3e + Robotiq 2F-140 + RealSense | Vegetable Sorting – ROS2 Humble

This repository contains the custom ROS2 packages developed for integrating a UR3e robot arm, Robotiq 2F-140 gripper, and Intel RealSense D435i into a unified system for a vegetable sorting project.
It includes custom description files, launch files, and initial motion-planning scripts.

External dependencies (UR packages, Robotiq driver, RealSense ROS, MoveIt config, UR driver) are intentionally excluded and must be installed separately.

📦 Repository Contents
1. ur3e_robotiq_description/

Custom URDF/Xacro definitions that extend:

the official UR3e description

the Robotiq 2F-140 meshes

our combined robot + gripper model

RViz configs for visualization

Includes:

ur3e_with_robotiq.urdf.xacro

ur3e_robotiq_macro.xacro

ur_config.rviz

2. ur3e_robotiq_launch/

Launch files for:

RViz visualization

Model loading

(Upcoming) MoveIt motion planning

(Upcoming) Pick-and-place pipeline

Includes:

view_ur3e_robotiq.launch.py

3. ur3e_robotiq_pickplace/

Python MoveIt2 scripts for:

connecting to MoveIt’s move_group

simple motion commands

placeholder for gripper control service

(Current scripts are placeholders; final implementation continues next milestone.)

🛠 External Dependencies (Not included in repo)

Clone these separately in your workspace:

ur_description
ur_moveit_config
Universal_Robots_ROS2_Driver
robotiq_2f_gripper_ros2
realsense-ros


Also required:

ROS2 Humble

MoveIt 2

Intel RealSense SDK / librealsense

📂 Workspace Structure (Recommended)
ur3e_robotiq_ws/
│
├── src/
│   ├── ur3e_robotiq_description/       ✓ in repo
│   ├── ur3e_robotiq_launch/            ✓ in repo
│   ├── ur3e_robotiq_pickplace/         ✓ in repo
│   │
│   ├── external packages (ignored)
│
├── build/   (ignored)
├── install/ (ignored)
├── log/     (ignored)

🚀 Running
Visualize robot:
ros2 launch ur3e_robotiq_launch view_ur3e_robotiq.launch.py

(Upcoming) MoveIt Launch:
ros2 launch ur3e_robotiq_launch moveit.launch.py

📝 Project Status (Milestone 2 – Completed)
Achieved:

Integrated UR3e + Robotiq gripper in ROS2

Full combined URDF validated

Working RViz visualization

Initial MoveIt environment running

URDF debugging + Xacro fixes

Workspace cleaned + version-controlled

Next Steps:

Add gripper joint to MoveIt SRDF

Integrate classification + depth (team modules)

Full pick-place motion pipeline

Real robot testing in January

👤 Author

Ruchit Bhanushali
M.Eng Robotics – DIT
Vegetable Sorting Group Project (UR3e + Computer Vision)
