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
RUN apt install python3 -y
