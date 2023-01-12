# Copyright (c) 2021  IBM Corporation, Carnegie Mellon University, and others
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

ARG ROS_DISTRO
ENV ROS_DISTRO=${ROS_DISTRO}
ARG TZ="Etc/UTC"
ENV TZ=$TZ
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

## install dependencies

RUN apt update && \
    apt-get install -y \
	iw \
        python3-colcon-common-extensions \
	python3-pip \
	python3-gi \
	python3-dbus \
	python-is-python3 \
	ros-${ROS_DISTRO}-diagnostic-updater \
	ros-${ROS_DISTRO}-tf2-ros \
        sudo \
    && \
    rm -rf /var/lib/apt/lists/*

RUN pip3 install --no-cache \
         pyserial

ENV OVERLAY_WS=/opt/overlay_ws
ENV INSTALL_DIR=$OVERLAY_WS/src
ENV PACKAGE=wireless_scanner_ros

## install pip and npm dependencies
RUN mkdir -p $INSTALL_DIR/$PACKAGE

WORKDIR $INSTALL_DIR/$PACKAGE

## copy source
ADD . $INSTALL_DIR/$PACKAGE

## catkin_make
WORKDIR $OVERLAY_WS
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    colcon build

## set wifi interface name
ARG WIFI_INTERFACE=wlan0
ENV WIFI_INTERFACE=$WIFI_INTERFACE

# install entrypoint
COPY docker-entrypoint.sh /usr/local/bin/
RUN ln -s /usr/local/bin/docker-entrypoint.sh

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

USER $USERNAME

ENV HOME /home/$USERNAME

ENTRYPOINT [ "docker-entrypoint.sh" ]
