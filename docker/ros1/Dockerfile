# Copyright (c) 2020  Carnegie Mellon University
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

ARG FROM_IMAGE
FROM $FROM_IMAGE AS cache

ARG RELEASE
ARG ROS_DISTRO
ARG TZ="Etc/UTC"

ENV RELEASE=${RELEASE:-focal} \
	ROS_DISTRO=${ROS_DISTRO:-noetic} \
	TZ=$TZ \
	DEBIAN_FRONTEND="noninteractive" \
	CMAKE_VERSION=3.14 \
	CMAKE_BUILD=7

RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

## install ubuntu packages
RUN apt update && apt install -q -y --no-install-recommends \
	curl \
	dirmngr \
	dnsutils \
	git \
	gpg-agent \
	libalut-dev \
	libarmadillo-dev \
	libboost-python-dev \
	libignition-math4-dev \ 
	libpcap-dev \
	libxi-dev \
	libxmu-dev \
	lsb-release \
	mesa-utils \
	mongodb \
	mongodb-dev \
	python3-pip \
	scons \
	software-properties-common \
	sox \
	wget \
	sysstat \
	&& \
	apt clean && \
	rm -rf /var/lib/apt/lists/*

## install ROS

RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $RELEASE main" > /etc/apt/sources.list.d/ros-latest.list' && \
	apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 && \
	apt update && \
	apt -q -y --allow-unauthenticated install ros-$ROS_DISTRO-desktop-full --no-install-recommends \
	&& \
	rm -rf /var/lib/apt/lists/*

RUN apt update && \
	apt -q -y install \
	ros-$ROS_DISTRO-navigation \
	ros-$ROS_DISTRO-urg-node \
	ros-$ROS_DISTRO-hector-mapping \
	ros-$ROS_DISTRO-geographic-msgs \
	ros-$ROS_DISTRO-rosserial \
	ros-$ROS_DISTRO-teleop-twist-joy \
	ros-$ROS_DISTRO-teleop-twist-keyboard \
	ros-$ROS_DISTRO-tf2-sensor-msgs \
	ros-$ROS_DISTRO-people-msgs \
	ros-$ROS_DISTRO-gmapping \
	ros-$ROS_DISTRO-realsense2-description \
	&& \
# need to upgrade to avoid gazebo error
	apt upgrade -y libignition-math4 && \
	apt clean && \
	rm -rf /var/lib/apt/lists/*

RUN apt update && \
	apt install -y --no-install-recommends \
	bluetooth \
	bluez \
	bluez-tools \
	rfkill \
	net-tools \
	pulseaudio-utils \
	bc \
	&& \
	apt clean && \
	rm -rf /var/lib/apt/lists/*
    
RUN pip3 install --upgrade pip && \
	pip install --no-cache-dir \
	coverage==5.5

RUN pip3 install --upgrade pip && \
	pip install --no-cache-dir \
        future \
	fastdtw \
	ibm_watson==3.4.2 \
	pymongo==3.12.2 \
	pyproj \
	pyserial \
	monotonic \
	IPython==5.0 \
	pygatt \
	gatt \	
	scipy

# install odrive
RUN pip3 install --no-cache-dir \
        odrive==0.5.2.post0 

# audio output
RUN apt update && \
	apt install -y --no-install-recommends \
	python-is-python3 \
	python3-rosdep \
	pulseaudio \
	&& \
	apt clean && \
	rm -rf /var/lib/apt/lists/*

COPY pulse-client.conf /etc/pulse/client.conf

# mesa driver
RUN apt update && \
	apt install -y --no-install-recommends \
	mesa-utils \
	xserver-xorg-video-all \
	&& \
	apt clean && \
	rm -rf /var/lib/apt/lists/*

RUN apt update && \
	apt install -y --no-install-recommends \
	build-essential \
	python3-tk \
	&& \
	apt clean && \
	rm -rf /var/lib/apt/lists/*

ENV USERNAME developer

# Replace 1000 with your user/group id
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



# install entrypoint
COPY docker-entrypoint.sh /usr/local/bin/
COPY launch.sh /usr/local/bin/

RUN ln -s /usr/local/bin/docker-entrypoint.sh / && \
    ln -s /usr/local/bin/launch.sh /

USER $USERNAME


ENV HOME /home/$USERNAME

WORKDIR $HOME/catkin_ws

ENTRYPOINT [ "docker-entrypoint.sh" ]
CMD [ "launch.sh" ]
#CMD [ "/bin/bash" ]
