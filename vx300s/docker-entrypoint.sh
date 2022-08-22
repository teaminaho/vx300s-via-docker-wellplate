#!/bin/bash -e

# Setup catkin workspace
. ~/interbotix_ws/devel/setup.bash

# Start ros nodes
# roslaunch --no-summary --screen --wait interbotix_xsarm_descriptions xsarm_description.launch robot_model:=vx300s use_joint_pub_gui:=true
roslaunch --no-summary --screen --wait interbotix_xsarm_control xsarm_control.launch robot_model:=vx300s use_sim:=false
# roslaunch --no-summary --screen --wait interbotix_xsarm_perception xsarm_perception.launch robot_model:=vx300s use_pointcloud_tuner_gui:=true use_armtag_tuner_gui:=true
