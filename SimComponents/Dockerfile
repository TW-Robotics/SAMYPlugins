FROM ubuntu:20.04

# LABEL about the custom image
LABEL maintainer="leber@technikum-wien.at"
LABEL version="0.1"
LABEL description="This is custom Docker Image for \
running the SAMY Python Template"

ARG DEBIAN_FRONTEND=noninteractive
ENV IP_ADDRESS=172.0.0.2

## installing dependencies
RUN apt update
RUN apt install -y python3 build-essential python3-pip
RUN pip3 install ipython python-dateutil pytz lxml cryptography pypubsub numpy

## copy files into image
RUN mkdir /usr/src/samy
WORKDIR /usr/src/samy
COPY . ./
## Install custom opcua package and saymplugin package
RUN pip3 install pip/opcua-0.98.13-py3-none-any.whl 
RUN pip3 install pip/samyplugin-0.3-py3-none-any.whl
