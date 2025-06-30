## Description

This repository hosts the lio_sam package, adapted with specific modifications to ensure compatibility with ROS 2 and integration into this point cloud generation project.

Tested on: 30 June 2025

## What is LIO-SAM?

LIO-SAM is a framework designed for highly accurate, real-time trajectory estimation and map-building for mobile robots. It builds lidar-inertial odometry using a factor graph approach, enabling the integration of various relative and absolute measurements—such as loop closures—from multiple sources as factors within the system.

Refer to the original documentation of LIO-SAM Algorithm through this repository:
https://github.com/TixiaoShan/LIO-SAM

## Configuration

Clone this repository to use lio_sam with ros2 projects. 

Make modifications to `config/params.yaml` according to the robot specs.

To save results as .pcd files, make sure savePCD is set to true and ensure that the savePCDDirectory exists. 

Also, make sure that the topic names and frame IDs match your robot's configuration. (replacing params.yaml will achieve everything above)

