ARG FROM_IMAGE

FROM $FROM_IMAGE as base

ENV ROS_DISTRO=humble

RUN apt update && apt install -q -y --no-install-recommends \
    build-essential \
    git \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-pip \
    python3-rosdep \
    python3-vcstool \
    && \
    apt clean && \
    rm -rf /var/lib/apt/lists/* && \
    rosdep init

RUN colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml && \
    colcon mixin update default

RUN apt update && \
    apt install -y --no-install-recommends \
    libasio-dev \
    libpcap-dev \
    ros-$ROS_DISTRO-camera-info-manager \
    ros-$ROS_DISTRO-cyclonedds \
    ros-$ROS_DISTRO-rmw-cyclonedds-cpp \
    ros-$ROS_DISTRO-gazebo-dev \
    ros-$ROS_DISTRO-diagnostic-updater \
    ros-$ROS_DISTRO-osrf-testing-tools-cpp \
    ros-$ROS_DISTRO-test-msgs \
    && \
    rm -rf /var/lib/apt/lists/*

FROM base as cache

ENV CUSTOM_DIR=/opt/custom_ws

RUN mkdir -p $CUSTOM_DIR/src
COPY . $CUSTOM_DIR/src

WORKDIR $CUSTOM_DIR

RUN rosdep update && rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -y --skip-keys "xacro librealsense2"

RUN . /opt/ros/humble/setup.sh && \
	colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release  # --executor sequential

FROM base as build

ENV CUSTOM_DIR=/opt/custom_ws

RUN mkdir -p $CUSTOM_DIR/install
COPY --from=cache $CUSTOM_DIR/install $CUSTOM_DIR/install
RUN sed -i "s:/ros/\$ROS_DISTRO/:/custom_ws/install/:" /ros_entrypoint.sh

