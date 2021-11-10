FROM ubuntu:18.04
# LABEL about the custom image
LABEL maintainer="leber@technikum-wien.at"
LABEL version="0.1"
LABEL description="This is custom Docker Image for \
running the Doosan SAMY Plugin"

ARG DEBIAN_FRONTEND=noninteractive

## installing dependencies
RUN apt update
RUN apt install software-properties-common -y
RUN add-apt-repository ppa:mhier/libboost-latest
RUN apt update && apt upgrade -y
RUN apt remove libboost1.65 libboost1.65-dev
RUN apt install libboost1.70 libboost1.70-dev
RUN apt install -y  build-essential \
 cmake nano libyaml-cpp-dev git libspdlog-dev \
 libpocofoundation50 libpocoutil50 libpoconet50 libpoco-dev libeigen3-dev

## copy files into image
RUN mkdir /usr/src/samy
WORKDIR /usr/src/samy
COPY . ./
# build and install open62541
RUN rm -r open62541_v1.2_unions_fixed/build
RUN mkdir open62541_v1.2_unions_fixed/build
WORKDIR open62541_v1.2_unions_fixed/build
RUN cmake -DUA_NAMESPACE_ZERO=FULL \
  	-DUA_ENABLE_SUBSCRIPTIONS=ON \
  	-DUA_ENABLE_SUBSCRIPTIONS_EVENTS=ON \
	-DUA_ENABLE_DISCOVERY=ON \
	-DUA_ENABLE_DISCOVERY_MULTICAST=ON \
	-DUA_ENABLE_DA=ON \
	-DUA_ENABLE_METHODCALLS=ON \
	-DUA_ENABLE_NODEMANAGEMENT=ON \
	-DUA_ENABLE_MICRO_EMB_DEV_PROFILE=OFF \
	-DUA_ENABLE_PUBSUB=ON \
	-DUA_ENABLE_PUBSUB_ETH_UADP=ON \
	-DUA_ENABLE_PUBSUB_ETH_UADP_ETF=ON \
	-DUA_ENABLE_PUBSUB_FILE_CONFIG=ON \
	-DUA_ENABLE_PUBSUB_INFORMATIONMODEL=ON \
	-DUA_ENABLE_PUBSUB_INFORMATIONMODEL_METHODS=ON \
	-DUA_ENABLE_PUBSUB_MONITORING=ON \
	-DUA_ENABLE_PUBSUB_MQTT=ON \
	-DUA_ENABLE_JSON_ENCODING=ON \
	..
RUN make -j
RUN make install

RUN useradd -ms /bin/bash samy
USER samy
RUN mkdir /home/samy/src/
WORKDIR /home/samy/src


## build SAMY Core
# WORKDIR /usr/src/samy
# RUN rm -r build && mkdir build
# WORKDIR build
# RUN cmake ..
# RUN make -j
