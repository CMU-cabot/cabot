ARG FROM_IMAGE

FROM $FROM_IMAGE

RUN mkdir -p /opt/cartographer_ws/src
COPY ./cartographer /opt/cartographer_ws/src/cartographer
COPY ./cartographer_ros /opt/cartographer_ws/src/cartographer_ros

WORKDIR /opt/cartographer_ws/

RUN apt update && rosdep update && rosdep install -q -y \
	--from-paths src \
	--ignore-src

## TODO: this is a quick fix
RUN sed -i 's:focal:jammy:' src/cartographer/scripts/install_debs_cmake.sh
RUN sed -i 's:d902eb869bcfacc1bad14933ed9af4bed006d481:8c0b94e793a66495e0b1f34a5eb26bd7dc672db0:' src/cartographer/scripts/install_abseil.sh
## TODO end

RUN src/cartographer/scripts/install_debs_cmake.sh && rm -rf /var/lib/apt/lists/*
RUN src/cartographer/scripts/install_abseil.sh && rm -rf /var/lib/apt/lists/*

RUN . /opt/ros/humble/setup.sh && \
	colcon build

ARG ROS_DISTRO
RUN sed -i 's:ros/$ROS_DISTRO:cartographer_ws/install:' /ros_entrypoint.sh

RUN apt update && \
    apt install -y --no-install-recommends \
    ros-${ROS_DISTRO}-rosbridge-suite && \
    apt install -y --no-install-recommends \
    nodejs \
    npm \
    python3-pip \
    ros-${ROS_DISTRO}-tf2-geometry-msgs \
    ros-${ROS_DISTRO}-gazebo-msgs \
    ros-${ROS_DISTRO}-velodyne-description \
    ros-${ROS_DISTRO}-realsense2-description \
    && \
    apt clean && \
    rm -rf /var/lib/apt/lists/*

RUN apt update && \
    apt install -y --no-install-recommends \
    ros-${ROS_DISTRO}-tf-transformations \
    ros-${ROS_DISTRO}-diagnostic-updater \
    libpcap-dev \
    python3-matplotlib \
    python3-sklearn \
    && \
    apt clean && \
    rm -rf /var/lib/apt/lists/*

RUN pip install --no-cache-dir \
    requests \
    certifi \
    pyproj \
    orjson

## install cvi client packages
RUN apt update && \
    apt install -y --no-install-recommends \
    python3-tk \
    python-is-python3 \
    && \
    apt clean && \
    rm -rf /var/lib/apt/lists/*

## install packages for ublox node and RTKLIB
RUN apt update && \
    apt-get install -y \
    libasio-dev \
    ros-${ROS_DISTRO}-rtcm-msgs \
    ros-${ROS_DISTRO}-nmea-msgs \
    gfortran \
    && \
    rm -rf /var/lib/apt/lists/*

## install RTKLIB
WORKDIR /opt
COPY ./RTKLIB/ ./RTKLIB/
RUN (cd RTKLIB/lib/iers/gcc/; make) && \
    (cd RTKLIB/app/consapp; make; make install) && \
    rm -rf RTKLIB

# TODO merge
RUN pip install --no-cache-dir \
	transforms3d 

COPY ./launch.sh /
COPY ./launch_rtklib.sh /

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
WORKDIR $HOME/loc_ws
