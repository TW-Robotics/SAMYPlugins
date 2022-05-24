#!/bin/bash
echo "Starting ROS and sleeping for 5 seconds..."
xterm -e roslaunch dsr_launcher dsr_moveit_gazebo.launch &
sleep 5
echo "Starting main.py"
python3 main.py 127.0.0.1 doosan samyConfigFiles/ 
