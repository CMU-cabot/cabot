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

FROM ubuntu:20.04

# install ros2 galactic packages
# https://index.ros.org/doc/ros2/Installation/Foxy/Linux-Development-Setup/

ARG TZ="Etc/UTC"

ENV LANG=en_US.UTF-8 \
	TZ=$TZ \
	DEBIAN_FRONTEND="noninteractive" \
        ROS_DISTRO="galactic"

RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

RUN apt update && \
	apt install -y curl gnupg2 lsb-release locales software-properties-common && \
	add-apt-repository universe && \
	curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg && \
#	curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - && \
	echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null && \
#	sh -c 'echo "deb http://packages.ros.org/ros2-testing/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list' && \
	locale-gen en_US en_US.UTF-8 && \
	update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && \
	rm -rf /var/lib/apt/lists/*


RUN apt update && apt install -y \
	build-essential \
	cmake \
	git \
	libbullet-dev \
	python3-colcon-common-extensions \
        python3-colcon-mixin \
	python3-flake8 \
	python3-pip \
	python3-pytest-cov \
	python3-rosdep \
	python3-setuptools \
	python3-vcstool \
	wget && \
# install some pip packages needed for testing
	python3 -m pip install -U \
	argcomplete \
	flake8-blind-except \
	flake8-builtins \
	flake8-class-newline \
	flake8-comprehensions \
	flake8-deprecated \
	flake8-docstrings \
	flake8-import-order \
	flake8-quotes \
	pytest-repeat \
	pytest-rerunfailures \
	pytest && \
# install Fast-RTPS dependencies
	apt install --no-install-recommends -y \
	libasio-dev \
	libtinyxml2-dev && \
# install CycloneDDS dependencies
	apt install --no-install-recommends -y \
	libcunit1-dev \
	libssl-dev

RUN mkdir -p /opt/ros/src
WORKDIR /opt/ros

# foxy is not ready yet, specify the master as of May 1, 2020 
#RUN wget https://raw.githubusercontent.com/ros2/ros2/foxy/ros2.repos && \

COPY ros2.repos /opt/ros
RUN vcs import src < ros2.repos

RUN apt update && \
	rosdep init && \
	rosdep update && \
	rosdep install --from-paths src --ignore-src --rosdistro galactic -y --skip-keys "console_bridge fastcdr fastrtps rti-connext-dds-5.3.1 urdfdom_headers" && \
	rm -rf /var/lib/apt/lists/* && \
	rm -rf /root/.cache/pip

#RUN colcon build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo --install-base foxy
RUN colcon build --install-base galactic --cmake-args " -DCMAKE_BUILD_TYPE=Release"

RUN colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml && \
	colcon mixin update default

COPY ros_entrypoint.sh /
ENTRYPOINT [ "/ros_entrypoint.sh" ] 
