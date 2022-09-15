#!/usr/bin/env bash
set -x

set -e
[ "x${HOME}" != "x" ]
[ -e "/app/arm_apps/arm_ctrl/launch/arm_ctrl.launch" ]
[ -x "/app/arm_apps/arm_ctrl/scripts/wellplate_picking.py" ]

. /opt/ros/noetic/setup.bash
. "${HOME}"/interbotix_ws/devel/setup.bash

which catkin_create_pkg >/dev/null 2>&1
which catkin_make >/dev/null 2>&1
which sed >/dev/null 2>&1

cd "${HOME}"/interbotix_ws/src

mkdir -p ./arm_apps
cd arm_apps
cp -r /app/arm_apps/arm_ctrl/ ./
cp -r /app/arm_apps/arm_ctrl_msgs/ ./
cp -r /app/arm_apps/arm_ctrl_rviz/ ./

cd "${HOME}"/interbotix_ws
catkin_make && catkin_make install

set +e
