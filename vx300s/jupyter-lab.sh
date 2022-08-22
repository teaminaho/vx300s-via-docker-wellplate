#!/usr/bin/env bash

. /opt/ros/noetic/setup.bash
. ${HOME}/interbotix_ws/devel/setup.bash

jupyter-lab --allow-root --notebook-dir=/jupyter
