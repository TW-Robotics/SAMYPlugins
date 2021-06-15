# Download base image ubuntu 14.04
FROM ubuntu:20.04

# LABEL about the custom image
LABEL maintainer="leber@technikum-wien.at"
LABEL version="0.1"
LABEL description="This is custom Docker Image for \
running the SAMY ABB Plugin"

ARG DEBIAN_FRONTEND=noninteractive
ENV IP_ADDRESS=172.0.0.2

## installing dependencies
RUN apt update
RUN apt install -y libboost-python1.71.0 build-essential cmake libyaml-cpp-dev libboost-all-dev python3-pip
RUN pip3 install opcua cryptography numpy

## copy files into image
RUN mkdir /usr/src/samy
WORKDIR /usr/src/samy
COPY . ./
## install open62541
WORKDIR open62541_v1.2_unions_fixed/build
RUN cmake -DUA_NAMESPACE_ZERO=FULL -DUA_ENABLE_SUBSCRIPTIONS=ON -DUA_ENABLE_SUBSCRIPTIONS_EVENTS=ON \
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
## start plugin (get ip address from docker)
WORKDIR /usr/src/samy
EXPOSE 4840

#ENTRYPOINT ["tail"]
#CMD ["-f","/dev/null"]
CMD /usr/src/samy/SAMYPlugIn_Base ${IP_ADDRESS}



