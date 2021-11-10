# Download base image ubuntu 14.04
FROM ros:melodic-robot
# LABEL about the custom image
LABEL maintainer="leber@technikum-wien.at"
LABEL version="0.1"
LABEL description="This is custom Docker Image for \
running the Doosan ROS package"

ARG DEBIAN_FRONTEND=noninteractive

RUN rosdep update

RUN apt install git
RUN mkdir /usr/src/ros
WORKDIR /usr/src/ros
RUN mkdir catkin_ws
RUN mkdir catkin_ws/src
RUN cd catkin_ws/src
RUN git clone https://github.com/doosan-robotics/doosan-robot
RUN rosdep install --from-paths doosan-robot --ignore-src --rosdistro melodic -r -y
RUN cd ..
RUN catkin_make
RUN source ./devel/setup.bash
