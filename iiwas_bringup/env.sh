#!/bin/bash

source /opt/ros/kinetic/setup.bash
source ~/catkin_ws/devel/setup.bash  #TODO change use your own workspace

export ROS_IP=192.168.2.4
# export ROS_MASTER_URI=http://192.168.2.37:11311  # IP of fermat
export ROS_MASTER_URI=http://192.168.2.2:11311   # IP of legendre

exec "$@"
