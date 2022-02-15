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
FROM $FROM_IMAGE

ENV ROS1_DISTRO noetic
ENV ROS2_DISTRO galactic

#######################################################################################
# setup base of bridge image : referred eloquent melodic ros1 bridge image
# https://github.com/osrf/docker_images/blob/master/ros/eloquent/ubuntu/bionic/ros1-bridge/Dockerfile

# setup keys
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# setup sources.list
RUN echo "deb http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros1-latest.list

# install ros1 packages
RUN apt update && apt install -y --no-install-recommends \
    ros-${ROS1_DISTRO}-ros-comm \
    ros-${ROS1_DISTRO}-roscpp-tutorials \
    ros-${ROS1_DISTRO}-rospy-tutorials \
    && \
    apt clean && \
    rm -rf /var/lib/apt/lists/*
#######################################################################################

# install other ros1 packages
RUN apt update && apt install -y --no-install-recommends \
	net-tools \
	iputils-ping \
	ros-${ROS1_DISTRO}-ros-base \
	ros-${ROS1_DISTRO}-tf2-msgs \
	ros-${ROS1_DISTRO}-rosgraph-msgs \
	ros-${ROS1_DISTRO}-trajectory-msgs \
	ros-${ROS1_DISTRO}-actionlib \
	ros-${ROS1_DISTRO}-actionlib-tutorials \
	ros-${ROS1_DISTRO}-control-msgs \
	ros-${ROS1_DISTRO}-roscpp \
	ros-${ROS1_DISTRO}-move-base-msgs \
	&& \
	apt clean && \
	rm -rf /var/lib/apt/lists/*

## build ros1 messages (people-msgs binary package for noetic is missing)
ENV OVERLAY_ROS1_MSG_WS /opt/overlay_ros1_msg_ws
RUN mkdir -p $OVERLAY_ROS1_MSG_WS/src
COPY ros1/people/people_msgs $OVERLAY_ROS1_MSG_WS/src/people_msgs
COPY ros1/nav2_msgs $OVERLAY_ROS1_MSG_WS/src/nav2_msgs
COPY ros1/queue_msgs $OVERLAY_ROS1_MSG_WS/src/queue_msgs

WORKDIR $OVERLAY_ROS1_MSG_WS
RUN . /opt/ros/${ROS1_DISTRO}/setup.sh && \
	catkin_make_isolated --install

## build ros2 messages
ENV OVERLAY_ROS2_MSG_WS /opt/overlay_ros2_msg_ws
RUN mkdir -p $OVERLAY_ROS2_MSG_WS/src
COPY control_msgs/control_msgs $OVERLAY_ROS2_MSG_WS/src/control_msgs
COPY people/people_msgs $OVERLAY_ROS2_MSG_WS/src/people_msgs
COPY queue_msgs $OVERLAY_ROS2_MSG_WS/src/queue_msgs

WORKDIR $OVERLAY_ROS2_MSG_WS
RUN . /opt/overlay_ws/install/setup.sh && \
	colcon build --symlink-install --packages-skip ros1_bridge

## build ros1_bridge
ENV OVERLAY_BRIDGE_WS /opt/overlay_bridge_ws
RUN mkdir -p $OVERLAY_BRIDGE_WS/src
COPY ros1_bridge $OVERLAY_BRIDGE_WS/src/ros1_bridge

WORKDIR $OVERLAY_BRIDGE_WS
RUN	. /opt/ros/${ROS1_DISTRO}/setup.sh && \
	. $OVERLAY_ROS1_MSG_WS/install_isolated/setup.sh && \
	. /opt/overlay_ws/install/setup.sh && \
	. $OVERLAY_ROS2_MSG_WS/install/local_setup.sh && \
	colcon build --packages-select ros1_bridge --cmake-force-configure && \
	rm -rf ./build ./log


RUN apt update && \
	apt install -y --no-install-recommends \
	ros-galactic-example-interfaces \
	&& \
	apt clean && \
	rm -rf /var/lib/apt/lists/*


## copy config files
## may better to move yaml files into under the home
COPY bridge_topics.yaml bridge_topics_sim.yaml launch.sh docker-entrypoint.sh /

RUN chmod 705 /docker-entrypoint.sh
RUN chmod 705 /launch.sh

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

WORKDIR $HOME/bridge_ws

ENTRYPOINT [ "/docker-entrypoint.sh" ]
CMD ["/launch.sh"]
