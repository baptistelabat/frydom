FROM gvincke/inca-build:latest

RUN apt-get update
RUN apt-get install -y libboost-filesystem1.62-dev
RUN apt-get install -y libhdf5-100 libhdf5-cpp-100 libhdf5-dev
RUN apt-get install -y build-essential xserver-xorg-dev x11proto-xf86vidmode-dev libxxf86vm-dev mesa-common-dev libgl1-mesa-dev libglu1-mesa-dev libxext-dev libxcursor-dev freeglut3-dev
RUN apt-get install -y libirrlicht-dev libirrlicht-doc libirrlicht1.8 libirrlicht1.8-dbg
RUN apt-get install -y libcurl4-gnutls-dev


# https://stackoverflow.com/questions/44331836/apt-get-install-tzdata-noninteractive
RUN DEBIAN_FRONTEND=noninteractive apt-get install -y tzdata
