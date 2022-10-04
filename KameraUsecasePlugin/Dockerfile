# Download base image ubuntu 20.04
FROM ubuntu:20.04

# LABEL about the custom image
LABEL maintainer="michael.schebek@technikum-wien.at"
LABEL version="0.1"
LABEL description="KameraPlugin for custom FHTW UseCase"

ARG DEBIAN_FRONTEND=noninteractive

## installing dependencies
RUN apt update -y
RUN apt upgrade -y
RUN apt install -y build-essential python3 python3-pip apt-utils libgl1 libglib2.0-0 libusb-1.0-0-dev udev
#RUN apt install -y libopencv-dev python3-opencv
RUN pip3 install opencv-python-headless pyrealsense2 numpy pypubsub requests scipy ipython python-dateutil pytz lxml cryptography pyyaml

## copy files into image
RUN mkdir /usr/src/samy
WORKDIR /usr/src/samy
COPY . ./

## Install custom opcua package and saymplugin package
RUN pip3 install pip/opcua-0.98.13-py3-none-any.whl 
RUN pip3 install pip/samyplugin-0.2-py3-none-any.whl
RUN ./setup_udev_rules.sh


