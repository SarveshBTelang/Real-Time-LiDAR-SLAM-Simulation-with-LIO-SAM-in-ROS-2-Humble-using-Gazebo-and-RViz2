## What is LIO-SAM?

LIO-SAM is a framework designed for highly accurate, real-time trajectory estimation and map-building for mobile robots. It builds lidar-inertial odometry using a factor graph approach, enabling the integration of various relative and absolute measurements—such as loop closures—from multiple sources as factors within the system.

Refer to the documentation of LIO-SAM Algorithm through this repository:
https://github.com/TixiaoShan/LIO-SAM

## Configuration

Clone the above mentioned repository and replace `config/params.yaml` with the one from this repo.

To save results as .pcd files, make sure savePCD is set to true and ensure that the savePCDDirectory exists. 

Also, make sure that the topic names and frame IDs match your robot's configuration. (replacing params.yaml will achieve everything above)

