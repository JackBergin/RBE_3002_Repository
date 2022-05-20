#!/bin/bash

echo "starting next phases"

source ~/catkin_ws/devel/setup.bash

roslaunch rbe_3002_final save_map.launch

sleep 1 

rosnode kill /turtlebot3_slam_gmapping

sleep 1 

roslaunch rbe_3002_final 3rd_phase.launch

echo "next phase started"
