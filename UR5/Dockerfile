# Download base image ubuntu 14.04
FROM ubuntu:20.04

# LABEL about the custom image
LABEL maintainer="leber@technikum-wien.at"
LABEL version="0.1"
LABEL description="This is custom Docker Image for \
running the SAMY UR5 Plugin"

ARG DEBIAN_FRONTEND=noninteractive
#ENV IP_ADDRESS=172.0.0.2

## installing dependencies
RUN apt update
RUN apt install -y build-essential python3-pip
RUN pip3 install numpy math3d scipy ipython pubsub pypubsub python-dateutil pytz lxml cryptography
RUN pip3 install /home/felix/Desktop/SAMY/Git/SAMYPlugins_Templates/SAMYPlugins_Template_Python/dist/samyplugin-0.0.1-py3-none-any.whl

## copy files into image
RUN mkdir /usr/src/samy
WORKDIR /usr/src/samy
COPY . ./

## start plugin (get ip address from docker)
EXPOSE 4840

#ENTRYPOINT ["tail"]
#CMD ["-f","/dev/null"]
