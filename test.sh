#!/bin/bash

sudo killall rosmaster
sudo killall gzserver
sudo killall gzclient

roslaunch amcl_depth willow-with-turtlebot.launch
roslaunch amcl_depth rtab_mapping.launch simulation:=true args:="--delete_db_on_start"
roslaunch amcl_depth rtab_rviz.launch

# Save octomap
rosrun octomap_server octomap_saver -f octomap.ot octomap_full:=/rtabmap/octomap_full
# Save Point Cloud
rosrun pcl_ros pointcloud_to_pcd input:=/rtabmap/cloud_map
# Save 2D occupancy grid
rosrun map_server map_saver -f mymap

# View octomap
octovis octomap.ot
 
roslaunch turtlebot_teleop keyboard_teleop.launch


# changed/opt/ros/kinetic/share/turtlebot_navigation/param/dwa_local_planner_params.yaml max_vel=0.55 sim_time=4 
# changed/opt/ros/kinetic/share/turtlebot_navigation/param/navfn.. allow_unknown=true


roslaunch turtlebot_gazebo turtlebot_world.launch
roslaunch rtabmap_ros demo_turtlebot_mapping.launch simulation:=true args:="--delete_db_on_start"
roslaunch rtabmap_ros demo_turtlebot_rviz.launch

catkin build -G"Eclipse CDT4 - Unix Makefiles" -DCMAKE_BUILD_TYPE=Debug
# Catkin build Parameter for generating Eclipse Project with c++11 support
catkin build -G"Eclipse CDT4 - Unix Makefiles" -DCMAKE_BUILD_TYPE=Debug -DCMAKE_CXX_COMPILER_ARG1=-std=c++11 -D__cplusplus=201103L -D__GXX_EXPERIMENTAL_CXX0X__=1

