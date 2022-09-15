#!/bin/bash -e

# Setup catkin workspace
. ~/interbotix_ws/devel/setup.bash

# Start ros nodes
roslaunch --no-summary --screen --wait arm_ctrl_rviz rviz.launch robot_model:=vx300s $@
