# Real-Time LiDAR SLAM Simulation with LIO-SAM in ROS 2 Humble using Gazebo and RViz2

![Simulation Result Overview](results/final_result_overview.png)
![Simulation Environment](results/environment.png)

## Overview

This project demonstrates a real-time LiDAR-based SLAM (Simultaneous Localization and Mapping) simulation using the LIO-SAM algorithm in ROS 2 Humble. The simulation environment is built with Gazebo and visualized in RViz2. It uses a Velodyne LiDAR sensor model and a custom robot in a simulated world.

---

## Prerequisites

Ensure you have the following installed:

- ROS 2 Humble
- Gazebo (Fortress or compatible)
- RViz2
- colcon (ROS 2 build tool)
- CloudCompare (for .pcd file visualization, optional)

---

## Repository Structure & Dependencies

Clone the following repositories into the `src/` folder of your workspace (e.g., `lio_sam_gazebo_ros2/src`):

### A. LIO-SAM for ROS 2
- Repository: [https://github.com/TixiaoShan/LIO-SAM](https://github.com/TixiaoShan/LIO-SAM)
- Note: Replace the default `params.yaml` with the one provided in this repository.

### B. Velodyne Simulator
- Repository: [https://github.com/ToyotaResearchInstitute/velodyne_simulator](https://github.com/ToyotaResearchInstitute/velodyne_simulator)

---

## Build Instructions

```bash
cd lio_sam_gazebo_ros2
colcon build
# To limit memory usage:
colcon build --parallel-workers 1
```
Ensure all necessary dependencies are listed in the respective CMakeLists.txt files for each package in the src/ directory.

Before launching the build ,copy the required Gazebo models for the factory world to your ~/.gazebo/models/ directory, if not already present.

## Launch Instructions

1. 1st Terminal- Launch Gazebo Simulation
```bash
source lio_sam_gazebo_ros2/install/setup.bash

ros2 launch robot_gazebo robot_sim.launch.py
```
2. 2nd terminal
```bash
source lio_sam_gazebo_ros2/install/setup.bash

ros2 launch lio_sam run.launch.py
```
To save .pcd files make sure you have created directory for it in /home/<user>/Downloads/LOAM/
(see LIO-SAM-ros2/src/mapOptmization.cpp for more details.)

.pcd files can be analyzed with CloudCompare software or online LIDAR viewer tools.

## References
LIO-SAM-ros2: https://github.com/TixiaoShan/LIO-SAM
velodyne_simulator: https://github.com/ToyotaResearchInstitute/velodyne_simulator

https://www.youtube.com/@robotmania8896: https://youtu.be/NNR9RUNz5Pg?si=Ex7P1nzOOn515r1_
--> Migration to ROS 2 Humble with necessary bug fixes
--> Navigation using teleop_twist_keyboard in ros2 instead of joypad
--> Customized robot model and improved world env
