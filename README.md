# DepthLocalisation
***DepthLocalisation*** is an open source Stereo/RGBD camera and 3D LiDAR based localisation framework. This framework enables localisation in a premapped environment. 
Prior mapping can be achieved by using packages like [loam_velodyne](https://github.com/laboshinl/loam_velodyne) and [LeGO_LOAM](https://github.com/RobustFieldAutonomyLab/LeGO-LOAM). 
Visual Tracking is used to provide high frequency, low accuracy updates which are corrected by high accuracy depth residual optimization. 

This package is effectively an implementation of [this](https://ieeexplore.ieee.org/document/8594362) publication. This package is built on top of the ROS ecosystem.

This package has been tested on Ubuntu 16.04 & ROS kinetic.
## Installation
***premapped_localization*** depends on the following libraries:
  - [PCL](https://github.com/PointCloudLibrary/pcl) : [Installation instructions](https://github.com/ethz-asl/libpointmatcher/blob/master/doc/Compilation.md)

Installing this package this package:
```bash
cd ~/catkin_ws/src
git clone https://github.com/ShreyanshDarshan/premapped_localization.git

cd ~/catkin_ws/
catkin_make
```
