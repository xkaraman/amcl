#!/bin/bash

sudo killall rosmaster
sudo killall gzserver
sudo killall gzclient

roslaunch amcl_lib_pf willow-with-turtlebot.launch
roslaunch amcl_lib_pf rtab_mapping.launch simulation:=true args:="--delete_db_on_start"
roslaunch amcl_lib_pf rtab_rviz.launch

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

# Catkin build Parameter for generating Eclipse Project with c++11 support
catkin build -GEclipse CDT4 - Unix Makefiles -DCMAKE_BUILD_TYPE=Debug -DCMAKE_CXX_COMPILER_ARG1=-std=c++11 -D__cplusplus=201103L -D__GXX_EXPERIMENTAL_CXX0X__=1

# Modified Kobuki_description package, urdf/kobuki gazebo.urdf.xacro to provide ground truth data
roscd kobuki_description/urdf
sudo gedit kobuki_gazebo.urdf.xacro
# Add these lines between <xacro:macro>
	<gazebo>
		<plugin name="p3d_base_controller" filename="libgazebo_ros_p3d.so">
		<alwaysOn>true</alwaysOn>
		<updateRate>50.0</updateRate>
		<bodyName>base_link</bodyName>
		<topicName>ground_truth/state</topicName>
		<gaussianNoise>0.01</gaussianNoise>
		<frameName>map</frameName>
		<xyzOffsets>0 0 0</xyzOffsets>
		<rpyOffsets>0 0 0</rpyOffsets>
		</plugin>
	</gazebo>
	
# Topics to record
/ground_truth/state
/map
/map_metadata
/octomap_full
/odom
/scan
/amcl_depth/bestpose
/amcl_depth/particlecloud

#amcl_depth
rosbag record -O subset /ground_truth/state /odom /amcl_depth/bestpose /amcl_depth/particlecloud
#amcl stack
rosbag record -O amcl_stack /ground_truth/state /odom /amclpose 

# plot with evo
source activate rospyeval
evo_traj bag subset.bag /amcl_depth/bestpose /odom --ref=/ground_truth/state -p --plot_mode=xy
evo_ape bag subset.bag /ground_truth/state /amcl_depth/bestpose --plot --plot_mode xy --save_results results/amcl_depth.zip -va -r full

evo_ape bag amcl_stack.bag /ground_truth/state /amclpose --plot --plot_mode xy --save_results results/amcl_depth.zip -va -r full

# Relay amcl best_pose from PoseWithCovarianceStamped to PoseStamped
rosrun topic_tools relay_field /amcl_pose /amclpose geometry_msgs/PoseStamped "header:
  seq: m.header.seq            
  stamp:                     
    secs: m.header.stamp.secs
    nsecs: m.header.stamp.nsecs
  frame_id: m.header.frame_id
pose:                        
  position:                    
    x: m.pose.pose.position.x
    y: m.pose.pose.position.y
    z: m.pose.pose.position.z
  orientation:               
    x: m.pose.pose.orientation.x                  
    y: m.pose.pose.orientation.y                 
    z: m.pose.pose.orientation.z     
    w: m.pose.pose.orientation.w "


