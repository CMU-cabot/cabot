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

ARG from=nvidia-cuda10.2-cudnn7-devel-glvnd-runtime-ros-core-ubuntu16.04

FROM $from AS build
ARG ROS_DISTRO=noetic
ENV ROS_DISTRO=$ROS_DISTRO
ARG UBUNTU_DISTRO=focal
#ARG LIBREALSENSE_V=2.48.0-0~realsense0.4976

# install required packages
RUN apt update && \
	apt-get install -y \
	libopencv-dev \
	python3-pip \
	python3-vcstool \
	software-properties-common \
	unzip \
	usbutils \
	wget \
	rsync \
	&& \
	rm -rf /var/lib/apt/lists/*

RUN apt-key adv --keyserver keys.gnupg.net --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || \
	apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE && \
	add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $UBUNTU_DISTRO main" -u

# temporal fix for build issue
# https://github.com/IntelRealSense/librealsense/issues/9607
RUN apt update && apt-get install -y \
	librealsense2-udev-rules \ 
	librealsense2-utils\
	librealsense2-dev \
	librealsense2-dbg 

RUN	apt-get install -y \
	librealsense2-dkms; exit 0 && \
	rm -rf /var/lib/apt/lists/*


ENV UNDERLAY_WS=/opt/underlay_ws
RUN mkdir -p $UNDERLAY_WS/src
WORKDIR $UNDERLAY_WS

ARG REALSENSE_ROS_V=2.2.22

# clone realsense-ros source and install dependencies
RUN cd src && \
	wget https://github.com/IntelRealSense/realsense-ros/archive/${REALSENSE_ROS_V}.zip && \
	unzip ${REALSENSE_ROS_V}.zip && \
	cd .. && \
	apt update && \
	rosdep install --from-paths ./src --ignore-src --rosdistro $ROS_DISTRO -y && \
	rm -rf /var/lib/apt/lists/*

# copy customized RealSense setting
COPY ./rs_aligned_depth_1280x720_30fps.launch $UNDERLAY_WS/src/realsense-ros-${REALSENSE_ROS_V}/realsense2_camera/launch/

# catkin_make
WORKDIR $UNDERLAY_WS/
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
	catkin_make install && \
	sed -i 's:ros/$ROS_DISTRO:underlay_ws/install:g' /ros_entrypoint.sh

