# UR3e + Robotiq 2F-140 + RealSense | Vegetable Sorting – ROS2 Humble

This repository contains the custom ROS2 packages developed for integrating a **UR3e robot arm**, **Robotiq 2F-140 gripper**, and **Intel RealSense D435i** into a unified system for a vegetable-sorting application.  
It includes custom description files, launch files, and initial motion-planning scripts.

External dependencies (UR packages, Robotiq driver, RealSense ROS, MoveIt config, UR driver) are intentionally excluded and must be installed separately.

---

## Repository Contents

### 1. `ur3e_robotiq_description/`

Custom URDF/Xacro definitions that extend:

- official UR3e description  
- Robotiq 2F-140 meshes  
- full combined robot + gripper model  
- RViz configuration files  

Includes:

- `ur3e_with_robotiq.urdf.xacro`
- `ur3e_robotiq_macro.xacro`
- `ur_config.rviz`

---

### 2. `ur3e_robotiq_launch/`

Launch files for:

- RViz visualization  
- Model loading  
- (Upcoming) MoveIt motion planning  
- (Upcoming) Pick-and-place integration  

Includes:

- `view_ur3e_robotiq.launch.py`

---

### 3. `ur3e_robotiq_pickplace/`

Python MoveIt2 scripts for:

- connecting to the MoveIt `move_group` interface  
- basic motion commands  
- placeholder Robotiq gripper call  

(Current scripts will be extended in the next milestone.)

---

## External Dependencies  
(Not included in this repository)

Clone these separately into your workspace:

```bash
https://github.com/UniversalRobots/Universal_Robots_ROS2_Description.git
https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver.git
https://github.com/PickNikRobotics/ros2_robotiq_gripper.git
https://github.com/realsenseai/realsense-ros.git
```

Required software:

- ROS2 Humble  
- MoveIt 2  
- Intel RealSense SDK (`librealsense`)  

---

## Recommended Workspace Structure
```bash
ur3e_robotiq_ws/
│
├── src/
│ ├── ur3e_robotiq_description/ # included
│ ├── ur3e_robotiq_launch/ # included
│ ├── ur3e_robotiq_pickplace/ # included
│
│ ├── external packages (ignored)
│
├── build/ # ignored
├── install/ # ignored
├── log/ # ignored

```
---

## Running

### Visualize the robot model in RViz

```bash
ros2 launch ur3e_robotiq_description view_ur3e_robotiq.launch.py
```

### Launch MoveIt
```bash
ros2 launch ur3e_robotiq_moveit_config ur_moveit.launch.py   ur_type:=ur3e use_fake_hardware:=true launch_rviz:=true
```

## Project Status (Milestone 2)
### Achieved

- UR3e + Robotiq 2F-140 integration in ROS2
- Combined URDF validated and loaded fully
- RViz end-to-end visualization running
- MoveIt environment loading
- Multiple URDF/Xacro and path issues resolved
- Workspace structured and version-controlled

## Next Steps
- Add gripper joint to MoveIt SRDF
- Integrate team’s classification + depth modules
- Implement complete pick-and-place pipeline
- Perform full testing on real UR3e robot



## License
This project is released under the MIT License.
See the full license text in the LICENSE file.

Author
Ruchit Bhanushali
M.Eng Robotics – Deggendorf Institute of Technology
Vegetable Sorting Group Project (UR3e + Computer Vision)

---

# `LICENSE`

```text
MIT License

Copyright (c) 2025 Ruchit Bhanushali

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
