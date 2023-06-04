# TORM-updated

This repository contains an (unofficial) fork of the repo (TORM)[https://github.com/cheulkang/TORM] which accompanies the paper "TORM: Fast and Accurate Trajectory Optimization of Redundant Manipulator given an End-Effector Path". See that repo for more information. The changes that have been made are as follows:
- improved error reporting (max pose error in mm/deg)
- improved visualization
- improved setup instructions



## Setup

This all depends on ROS noetic and the corresponding moveit version

``` bash
mkdir -p ~/ros/cppflow_benchmarking_ws/src && cd ~/ros/cppflow_benchmarking_ws/src
git clone https://github.com/jstmn/TORM-updated
mv TORM-updated/ros_packages_you_need/fetch_description .  # from https://github.com/ZebraDevs/fetch_ros
mv TORM-updated/ros_packages_you_need/move_it_config_fetch .
cd ../ && catkin_make
# Optionall create a new moveit planning group. group name needs to be 'arm'
# roslaunch moveit_setup_assistant setup_assistant.launch
```

## Running
``` bash
# terminal 1
roscore

# terminal 2
rosparam load src/fetch_description/robots/fetch.urdf robot_description
roslaunch move_it_config_fetch move_group.launch

# terminal 3
rosrun rviz rviz -d src/TORM-updated/rviz_config.rviz

# terminal 4
roslaunch torm fetch_circle.launch # be sure to run this before 'rosrun torm main', you'll get a 'terminate called after throwing an instance of 'XmlRpc::XmlRpcException'' error otherwise
catkin_make && rosrun torm main
```



## Notes

The quaternion format is `(w, x, y, z)`