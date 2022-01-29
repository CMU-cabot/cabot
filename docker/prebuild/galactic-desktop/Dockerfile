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

ARG ROS_DISTRO=galactic
FROM ros:$ROS_DISTRO

# install ros2 galactic packages
# https://index.ros.org/doc/ros2/Installation/Foxy/Linux-Development-Setup/

ARG TZ="Etc/UTC"

ENV LANG=en_US.UTF-8 \
	TZ=$TZ \
	DEBIAN_FRONTEND="noninteractive" \
        ROS_DISTRO=$ROS_DISTRO

RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

RUN apt update && \
	apt install -y --no-install-recommends \
	curl \
	gnupg2 \
	lsb-release \
	locales \
	software-properties-common && \
	locale-gen en_US en_US.UTF-8 && \
	update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && \
	apt clean && \
	rm -rf /var/lib/apt/lists/*

RUN apt update && \
	apt install -y --no-install-recommends \
	ros-$ROS_DISTRO-rviz2 \
	&& \
	apt clean && \
	rm -rf /var/lib/apt/lists/*

COPY ros_entrypoint.sh /
ENTRYPOINT [ "/ros_entrypoint.sh" ] 
