# ABB PlugIn

## CRCLLib
In CRCLLib is a python implemantation of the CRCL language.

## TransformLib
In TransformLib is a library that provides functionf for common transformations. These are:
- Euler angles to homogeneos rotation matrix
- Homogeneos rotation matrix to euler angles
- Quaternions to homogeneos rotation matrix
- Homogeneos rotation matrix to Qaternions
Also it includes some helper functions:
- get_pose_euler: takes a pose with a homogeneos rotation matrix and returns a pose with euler angles.
- get_pose_quaternion: takes a pose with a homogeneos rotation matrix and returns a pose with quaternions.
- parse_waypoint: takes a waypoint and returns a pose

## RobotLib
Here is the robot speciic code. It provides all functions to communicate with the robot.

## plugin.py
In the plugin.py file is the code to handle the supported CRCL commands.

## SAMYPlugIn_Base
This is the C++ part of the SAMYPlugIn. It handles the communication between the SAMYCore and the SAMYPlugIn.
To start the SAMYPlugIn start this executable.
