# Download base image ubuntu 20.04
FROM ros:noetic

# LABEL about the custom image
LABEL maintainer="ma0180@technikum-wien.at"
LABEL version="0.2"
LABEL description="This is custom Docker Image for \
running the SAMY ROS Plugin"

ARG DEBIAN_FRONTEND=noninteractive

## installing dependencies
RUN apt update -y
RUN apt upgrade -y
RUN apt install -y build-essential python3-pip apt-utils 
RUN pip3 install requests numpy math3d scipy ipython pypubsub python-dateutil pytz lxml cryptography pyyaml pytransform3d
RUN apt-get install -y ros-noetic-rqt* ros-noetic-moveit* ros-noetic-gazebo-ros-control ros-noetic-joint-state-controller ros-noetic-effort-controllers ros-noetic-position-controllers ros-noetic-ros-controllers ros-noetic-ros-control ros-noetic-joint-state-publisher-gui ros-noetic-joint-state-publisher 
RUN apt install ros-noetic-serial* nano libx11-xcb-dev libglu1-mesa-dev libxrender-dev libxi-dev libxkbcommon-dev libxkbcommon-x11-dev -y

## copy files into image
RUN mkdir /usr/src/samy
WORKDIR /usr/src/samy
COPY . ./
RUN mkdir ~/catkin_ws
RUN mkdir ~/catkin_ws/src
RUN /bin/bash -c '. /opt/ros/noetic/setup.bash; cd  ~/catkin_ws; catkin_make'
WORKDIR /usr/src/samy
RUN cp -r doosan-robot ~/catkin_ws/src
RUN /bin/bash -c '. /opt/ros/noetic/setup.bash; cd  ~/catkin_ws; catkin_make'

## Install custom opcua package and saymplugin package
RUN pip3 install pip/opcua-0.98.13-py3-none-any.whl 
RUN pip3 install pip/samyplugin-0.3-py3-none-any.whl
