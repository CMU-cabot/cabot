ARG FROM_IMAGE

FROM $FROM_IMAGE as base

ENV ROS_DISTRO=humble

RUN apt update && \
    apt install -y --no-install-recommends \
    libasio-dev \
    libpcap-dev \
    ros-$ROS_DISTRO-camera-info-manager \
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

RUN . /opt/ros/humble/setup.sh && \
	colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release  # --executor sequential

FROM base as build

ENV CUSTOM_DIR=/opt/custom_ws

RUN mkdir -p $CUSTOM_DIR/install
COPY --from=cache $CUSTOM_DIR/install $CUSTOM_DIR/install

