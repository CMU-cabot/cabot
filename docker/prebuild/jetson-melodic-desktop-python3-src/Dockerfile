ARG from=nvcr.io/nvidia/l4t-base:r32.6.1

FROM $from

ENV DEBIAN_FRONTEND=noninteractive

# install ROS melodic using python 3
ENV ROS_DISTRO melodic

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
	apt-utils \
	curl \
	git \
	gnupg \
	gnupg1 \
        gnupg2 \
	lsb-release \
	pkg-config \
	python3 \
	python3-dev \
	python3-pip \
	python3-setuptools \
        python3-wstool \
	build-essential && \
# pip
    pip3 install -U --no-cache-dir \
	catkin_tools \
	catkin_pkg \
	rosdep \
	rospkg \
	rosinstall_generator \
	rosinstall \
	vcstools \
    && rm -rf /var/lib/apt/lists/* \
    && apt-get clean

RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' && \
    apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 && \
    curl -sSL 'http://keyserver.ubuntu.com/pks/lookup?op=get&search=0xC1CF6E31E6BADE8868B172B4F42ED6FBAB17C654' | apt-key add -

RUN pip3 install -U trollius --no-cache-dir \
	empy \
	gnupg \
	netifaces \
	rosdep \
	rospkg \
	rosinstall \
	rosinstall-generator

RUN apt-get update -y && apt-get install -y --no-install-recommends \
	pyqt5-dev \
	pyqt5-dev-tools \
	python3-defusedxml \
	python3-numpy \
	python3-pycryptodome \
	python3-pyqt5 \
	python3-sip-dev \
# to avoid error in rosdep install
	python-rospkg \
	python-catkin-pkg \
    && rm -rf /var/lib/apt/lists/* \
    && apt-get clean

RUN rosdep init && rosdep update

RUN mkdir /catkin_ws
WORKDIR /catkin_ws

RUN catkin config --init -DCMAKE_BUILD_TYPE=Release --blacklist rqt_rviz rviz_plugin_tutorials librviz_tutorial --install && \
	rosinstall_generator desktop --rosdistro $ROS_DISTRO --deps --tar > $ROS_DISTRO-desktop.rosinstall && \
# downgrade PyYAML to avoid Loader error with PyYAML 6.0+
	pip3 install PyYAML==5.4.1 --no-cache-dir && \
# build ros
	wstool init -j8 src $ROS_DISTRO-desktop.rosinstall && \
	wstool update -j4 -t src && \
	export ROS_PYTHON_VERSION=3 && \
	apt-get update && \
	apt upgrade -y && \
	rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y && \
	./src/catkin/bin/catkin_make_isolated --install \
	-DCMAKE_INSTALL_PREFIX=/opt/ros/$ROS_DISTRO \
	-DCMAKE_BUILD_TYPE=Release \
	-DPYTHON_EXECUTABLE=/usr/bin/python3 && \
	rm -rf /catkin_ws && \
	rm -rf /var/lib/apt/lists/* && \
	apt-get clean

# setup entrypoint
COPY ros_entrypoint.sh /
RUN chmod 777 /ros_entrypoint.sh

ENTRYPOINT [ "/ros_entrypoint.sh" ]
