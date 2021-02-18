# Copyright (c) 2021  IBM Corporation
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

ARG FROM_IMAGE=ros:noetic-ros-base-focal

FROM $FROM_IMAGE AS build
ARG ROS_DISTRO=noetic
ENV ROS_DISTRO=$ROS_DISTRO
ARG UBUNTU_DISTRO=focal
ARG GAZEBOV=11
ARG TZ="Etc/UTC"
ENV TZ=$TZ
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

#update gazebo to latest
# for kinetic
RUN apt update && \
	apt-get install -y \
	wget \
	&& \
	rm -rf /var/lib/apt/lists/*

RUN sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable $UBUNTU_DISTRO main" > /etc/apt/sources.list.d/gazebo-stable.list' && \
	wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add - && \
	apt-get update \
	&& \
	apt-get -q -y install gazebo$GAZEBOV libgazebo$GAZEBOV-dev \
	&& \
	rm -rf /var/lib/apt/lists/*
# for melodic
#RUN sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable bionic main" > /etc/apt/sources.list.d/gazebo-stable.list' && \
#	wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add - && \
#	apt-get update \
#	&& \
#	apt-get -q -y install gazebo9 libgazebo9-dev \
#	&& \
#	rm -rf /var/lib/apt/lists/*

# install required packages
# for kinetic
RUN apt update && \
	apt-get install -y \
	python-tk \
	ros-$ROS_DISTRO-people-msgs \
	ros-$ROS_DISTRO-tf2-geometry-msgs \
	ros-$ROS_DISTRO-move-base-msgs \
	ros-$ROS_DISTRO-gazebo-msgs \
	ros-$ROS_DISTRO-rviz \
	&& \
	rm -rf /var/lib/apt/lists/*
# for melodic
#RUN apt update && \
#	apt-get install -y \
#	python-tk \
#	xterm \
#	ros-$ROS_DISTRO-people-msgs \
#	ros-$ROS_DISTRO-tf2-geometry-msgs \
#	ros-$ROS_DISTRO-move-base-msgs \
#	ros-$ROS_DISTRO-gazebo-msgs \
#	ros-$ROS_DISTRO-rviz \
#	&& \
#	rm -rf /var/lib/apt/lists/*

# install cmake later than 3.12 to build darknet
# https://github.com/AlexeyAB/darknet#requirements
RUN wget https://github.com/Kitware/CMake/releases/download/v3.17.3/cmake-3.17.3.tar.gz && \
	tar xvzf cmake-3.17.3.tar.gz && \
	cd cmake-3.17.3 && \
	./bootstrap && \
	make && \
	make install

RUN apt update && \
	apt-get install -y \
	python3-pip \
	libfreetype6-dev \
	&& \
	rm -rf /var/lib/apt/lists/*
COPY requirements.txt /
RUN pip3 install -r /requirements.txt

COPY ./launch.sh ./resetusb.sh ./resetrs.sh /

# setup for display
ENV USERNAME developer

### replace 1000 with your user/group id
ARG UID=1000
RUN useradd -m $USERNAME && \
        echo "$USERNAME:$USERNAME" | chpasswd && \
        usermod --shell /bin/bash $USERNAME && \
	usermod -aG sudo $USERNAME && \
        mkdir -p /etc/sudoers.d/ && \
        echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers.d/$USERNAME && \
        chmod 0440 /etc/sudoers.d/$USERNAME && \
        usermod  --uid $UID $USERNAME && \
        groupmod --gid $UID $USERNAME

USER $USERNAME

ENV HOME /home/$USERNAME
WORKDIR $HOME/people_ws
