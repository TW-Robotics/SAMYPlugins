FROM ubuntu:20.04

# LABEL about the custom image
LABEL maintainer="leber@technikum-wien.at"
LABEL version="0.1"
LABEL description="This is custom Docker Image for \
running a simple OPCUA Server"

ARG DEBIAN_FRONTEND=noninteractive

## installing dependencies
RUN apt update
RUN apt install -y python3 build-essential python3-pip
RUN pip3 install ipython python-dateutil pytz lxml requests numpy pypubsub cryptography

## copy files into image
RUN mkdir /usr/src/samy
WORKDIR /usr/src/samy
COPY . ./
RUN pip3 install pip/opcua-0.98.13-py3-none-any.whl
RUN pip3 install pip/samyplugin-0.2-py3-none-any.whl

## start plugin (get ip address from docker)

#ENTRYPOINT ["tail"]
#CMD ["-f","/dev/null"]
CMD python3 /usr/src/samy/main.py
