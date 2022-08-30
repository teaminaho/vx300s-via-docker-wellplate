#!/usr/bin/env bash

. /opt/ros/noetic/setup.bash

rosrun usb_cam usb_cam_node _video_device:=/dev/video0 _pixel_format:=yuyv _framerate:=15

