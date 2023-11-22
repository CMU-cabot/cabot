# Copyright (c) 2023  Carnegie Mellon University, IBM Corporation, and others
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

ARG FROM_IMAGE=ros:noetic-ros-base-xenial

# multi-stage for caching
FROM $FROM_IMAGE AS cache

RUN apt update && \
    apt-get install -y python3-vcstool \
    && \
    rm -rf /var/lib/apt/lists/*

# clone underlay source (cartographer)
ENV UNDERLAY_WS /opt/underlay_ws
RUN mkdir -p $UNDERLAY_WS/src
WORKDIR $UNDERLAY_WS

COPY ./cartographer     ./src/cartographer
COPY ./cartographer_ros ./src/cartographer_ros

# copy overlay source (others)
ENV OVERLAY_WS /opt/overlay_ws
RUN mkdir -p $OVERLAY_WS/src
WORKDIR $OVERLAY_WS

COPY ./xsens_driver ./src/xsens_driver
COPY ./velodyne     ./src/velodyne

# copy manifests for caching
WORKDIR /opt
RUN mkdir -p /tmp/package && \
    mkdir -p /tmp/pip && \
    mkdir -p /tmp/npm
RUN find ./ -name "package.xml" | \
    xargs cp --parents -t /tmp/package
#RUN find ./ -name "requirements.txt" | \
#    xargs cp --parents -t /tmp/pip
#RUN find ./ -name "package.json" | \
#    xargs cp --parents -t /tmp/npm


FROM $FROM_IMAGE AS build
ARG ROS_DISTRO=noetic
ARG TZ="Etc/UTC"
ENV ROS_DISTRO=$ROS_DISTRO \
	DEBIAN_FRONTEND="noninteractive" \
	TZ=$TZ
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

RUN apt update && \
    apt-get install -y ninja-build git \
    stow \
    && \
    rm -rf /var/lib/apt/lists/*

# copy underlay manifests
COPY --from=cache /tmp/package /opt
ENV UNDERLAY_WS /opt/underlay_ws

# install proto3
WORKDIR $UNDERLAY_WS
COPY ./install_proto3_isolated.sh .
RUN ./install_proto3_isolated.sh

# install abseil
COPY ./cartographer/scripts/install_abseil.sh .
RUN ./install_abseil.sh

# install underlay dependencies
WORKDIR $UNDERLAY_WS
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    apt-get update && rosdep install -q -y \
      --from-paths src \
      --ignore-src \
    && rm -rf /var/lib/apt/lists/*

# copy underlay source
COPY --from=cache $UNDERLAY_WS ./

RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    PATH="${PWD}/protobuf/install/bin:${PATH}" \
    PROTOBUF_INCLUDE_DIRS="${PWD}/protobuf/install/include"  \
    PROTOBUF_LIBRARY="${PWD}/protobuf/install/lib/libprotobuf.a" \
    PROTOBUF_PROTOC_EXECUTABLE="${PWD}/protobuf/install/bin/protoc" \
    catkin_make_isolated --install --use-ninja \
    -DCMAKE_PREFIX_PATH="${PWD}/install_isolated;${CMAKE_PREFIX_PATH}"

RUN sed -i 's:ros/$ROS_DISTRO:underlay_ws/install_isolated:' /ros_entrypoint.sh

# install overlay dependencies
ENV OVERLAY_WS /opt/overlay_ws
WORKDIR $OVERLAY_WS
RUN . $UNDERLAY_WS/install_isolated/setup.sh && \
    apt-get update && rosdep install -q -y \
      --from-paths src \
      --ignore-src \
    && rm -rf /var/lib/apt/lists/*

# install packages for explore
# install ROS packages
RUN apt update && \
    apt-get install -y \
    ros-$ROS_DISTRO-navigation \
    ros-$ROS_DISTRO-tf2-sensor-msgs \
    ros-$ROS_DISTRO-vision-opencv \
    ros-$ROS_DISTRO-gazebo-msgs \
    ros-$ROS_DISTRO-xacro \
    ros-$ROS_DISTRO-robot-state-publisher \
    ros-$ROS_DISTRO-joint-state-publisher \
    ros-$ROS_DISTRO-pointcloud-to-laserscan \
	ros-$ROS_DISTRO-pcl-ros \
    && \
    rm -rf /var/lib/apt/lists/*
# install python packages
RUN apt update && \
    apt-get install -y \
    python3-pip \
    python3-networkx \
    python3-numba \
    python3-matplotlib \
    python3-opencv \
    python3-scipy \
    python3-skimage \
    python-is-python3 \
    && \
    rm -rf /var/lib/apt/lists/*

COPY ./launch.sh /

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
WORKDIR $HOME/explore_ws
