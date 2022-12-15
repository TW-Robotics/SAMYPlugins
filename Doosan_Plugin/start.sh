#!/bin/bash
echo "Starting ROS and sleeping for 5 seconds..."
source /opt/ros/noetic/setup.bash 
source ~/catkin_ws/devel/setup.bash 
roslaunch dsr_launcher samyros.launch robotip:="$1" robotport:="$2" robotmodel:="$3" robotmode:="$4" &
sleep 5
echo "Starting main.py"
python3 main.py localhost Doosan configFiles/
