# Download ROS noetic base image
FROM ros:noetic

# LABEL about the custom image
LABEL maintainer="ma0180@technikum-wien.at"
LABEL version="0.1"
LABEL description="This is custom Docker Image for \
running the SAMY ROS Plugin for the OpenManipulatorX"

ARG DEBIAN_FRONTEND=noninteractive

#Installing dependencies
RUN apt update && apt upgrade -y
RUN apt install -y build-essential python3-pip apt-utils git
RUN pip3 install requests numpy math3d scipy ipython pypubsub python-dateutil pytz lxml cryptography pyyaml pytransform3d
RUN apt install -y ros-noetic-ros-controllers ros-noetic-gazebo* ros-noetic-moveit* ros-noetic-industrial-core
RUN apt install -y ros-noetic-dynamixel-sdk ros-noetic-dynamixel-workbench*
RUN apt install -y ros-noetic-robotis-manipulator

#Creating Dirs and build OpenManipulator packages
RUN mkdir /usr/src/samy
WORKDIR /usr/src/samy
COPY . ./
# RUN mkdir catkin_ws
# RUN mkdir catkin_ws/src
# WORKDIR catkin_ws/src
# RUN git clone -b noetic-devel https://github.com/ROBOTIS-GIT/open_manipulator.git
# RUN git clone -b noetic-devel https://github.com/ROBOTIS-GIT/open_manipulator_msgs.git
# RUN git clone -b noetic-devel https://github.com/ROBOTIS-GIT/open_manipulator_simulations.git
# RUN git clone https://github.com/ROBOTIS-GIT/open_manipulator_dependencies.git
# RUN git clone https://github.com/ROBOTIS-GIT/open_manipulator_controls.git
WORKDIR /usr/src/samy/catkin_ws
RUN /bin/bash -c '. /opt/ros/noetic/setup.bash; cd  /usr/src/samy/catkin_ws; catkin_make'

#Copy config files
WORKDIR /usr/src/samy


## Install custom opcua package and saymplugin package
RUN pip3 install pip/opcua-0.98.13-py3-none-any.whl 
RUN pip3 install pip/samyplugin-0.2-py3-none-any.whl
