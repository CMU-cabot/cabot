FROM ubuntu:20.04

# install ros2 foxy packages
# https://index.ros.org/doc/ros2/Installation/Foxy/Linux-Development-Setup/

ARG TZ="Etc/UTC"

ENV LANG=en_US.UTF-8 \
	TZ=$TZ \
	DEBIAN_FRONTEND="noninteractive" \
        ROS_DISTRO="foxy"

RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

RUN apt update && \
	apt install -y curl gnupg2 lsb-release locales && \
	curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - && \
	sh -c 'echo "deb http://packages.ros.org/ros2-testing/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list' && \
	locale-gen en_US en_US.UTF-8 && \
	update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && \
	rm -rf /var/lib/apt/lists/*



# copy depends list from ros-elquent-desktop in http://packages.ros.org/ros2/ubuntu/dists/bionic/main/binary-amd64/Packages
# and remove some missing packages
#RUN apt update && \
#	apt install -y python3-vcstool git ros-foxy-action-tutorials-cpp ros-foxy-action-tutorials-interfaces ros-foxy-action-tutorials-py ros-foxy-composition ros-foxy-demo-nodes-cpp ros-foxy-demo-nodes-cpp-native ros-foxy-demo-nodes-py ros-foxy-dummy-map-server ros-foxy-dummy-robot-bringup ros-foxy-dummy-sensors ros-foxy-examples-rclcpp-minimal-action-client ros-foxy-examples-rclcpp-minimal-action-server ros-foxy-examples-rclcpp-minimal-client ros-foxy-examples-rclcpp-minimal-composition ros-foxy-examples-rclcpp-minimal-publisher ros-foxy-examples-rclcpp-minimal-service ros-foxy-examples-rclcpp-minimal-subscriber ros-foxy-examples-rclcpp-minimal-timer ros-foxy-examples-rclpy-executors ros-foxy-examples-rclpy-minimal-action-client ros-foxy-examples-rclpy-minimal-action-server ros-foxy-examples-rclpy-minimal-client ros-foxy-examples-rclpy-minimal-publisher ros-foxy-examples-rclpy-minimal-service ros-foxy-examples-rclpy-minimal-subscriber ros-foxy-image-tools ros-foxy-intra-process-demo ros-foxy-lifecycle ros-foxy-logging-demo ros-foxy-pendulum-control ros-foxy-pendulum-msgs ros-foxy-quality-of-service-demo-cpp ros-foxy-quality-of-service-demo-py ros-foxy-rviz-default-plugins ros-foxy-rviz2 ros-foxy-tlsf ros-foxy-tlsf-cpp ros-foxy-topic-monitor ros-foxy-ros-workspace && \
#	rm -rf /var/lib/apt/lists/*
#
#RUN apt update && \
#	apt install -y python3-rosdep && \
#	rosdep init && \
#	rosdep update && \
#	rm -rf /var/lib/apt/lists/*


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
#RUN wget https://raw.githubusercontent.com/ros2/ros2/0444bff7b23484fffdb78d170801e39cadf514f9/ros2.repos && \
RUN vcs import src < ros2.repos

RUN apt update && \
	rosdep init && \
	rosdep update && \
	rosdep install --from-paths src --ignore-src --rosdistro foxy -y --skip-keys "console_bridge fastcdr fastrtps rti-connext-dds-5.3.1 urdfdom_headers" && \
	rm -rf /var/lib/apt/lists/* && \
	rm -rf /root/.cache/pip

#RUN colcon build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo --install-base foxy
RUN colcon build --install-base foxy

RUN colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml && \
	colcon mixin update default

COPY ros_entrypoint.sh /
ENTRYPOINT [ "/ros_entrypoint.sh" ] 
