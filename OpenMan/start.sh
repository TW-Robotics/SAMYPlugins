#!/bin/bash
echo "Starting ROS and sleeping for 5 seconds..."
source /opt/ros/noetic/setup.bash 
source catkin_ws/devel/setup.bash 
roslaunch open_manipulator_controller open_manipulator_controller.launch baud_rate:=57600 &
sleep 5
echo "Starting main.py"
python3 main.py core OpenMan configFiles/
while [ true ] ; do
read -t 30 -n 1
if [ $? = 0 ] ; then
exit ;
else
echo ""
fi
done