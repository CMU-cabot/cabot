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
ENV ROS_DISTRO=humble

# TODO replace with rosdep install
RUN apt update && \
    apt install -y --no-install-recommends \
    bc \
    libqt5svg5-dev \
    libzmq3-dev \
    libdw-dev \
    libyaml-cpp-dev \
    libomp-dev \
    libpcl-dev \
    python3-pykdl \
    python3-sklearn \
    python3-venv \
    qtbase5-dev \
    ros-$ROS_DISTRO-diagnostic-aggregator \
    ros-$ROS_DISTRO-gazebo-msgs \
    ros-$ROS_DISTRO-joint-state-publisher \
    ros-$ROS_DISTRO-joint-state-publisher-gui \
    ros-$ROS_DISTRO-navigation2 \
# pcl-ros is not ported yet
# #    ros-$ROS_DISTRO-pcl-msgs \
# #    ros-$ROS_DISTRO-pcl-ros \
    ros-$ROS_DISTRO-pointcloud-to-laserscan \
    ros-$ROS_DISTRO-realsense2-description \
    ros-$ROS_DISTRO-robot-localization \
    ros-$ROS_DISTRO-geographic-msgs \
    libgeographic-dev \
# workaround end
    ros-$ROS_DISTRO-rosbridge-server \
    ros-$ROS_DISTRO-ros-testing \
    ros-$ROS_DISTRO-rqt-robot-monitor \
    ros-$ROS_DISTRO-teleop-twist-keyboard \
    ros-$ROS_DISTRO-tf-transformations \
    ros-$ROS_DISTRO-xacro \
    ros-$ROS_DISTRO-rmw-cyclonedds-cpp \
    ros-$ROS_DISTRO-rosbag2-storage-mcap \
    xterm \
    && \
    rm -rf /var/lib/apt/lists/*

RUN pip3 install --no-cache-dir \
    # install ipython==8.0.1 to prevent errors while using odrivetool
    ipython==8.0.1 \
    odrive==0.6.5 \
    pyproj \
    pyserial \
    requests \
    transforms3d

# for debug
# RUN apt update && \
#     apt install -y --no-install-recommends \
#     gdb \
#     vim \
#     && \
#     rm -rf /var/lib/apt/lists/*

RUN apt update && \
    apt install -y --no-install-recommends valgrind && \
    rm -rf /var/lib/apt/lists/*

# Check
RUN which valgrind

# set up venv for keeping compatibility
RUN mkdir -p /opt/venv
# cabot2 venv (odrive==0.5.2.post0)
RUN python3 -m venv --system-site-packages /opt/venv/cabot2 && \
    . /opt/venv/cabot2/bin/activate && \
    pip3 install --no-cache-dir odrive==0.5.2.post0 && \
    deactivate
# cabot3 venv (odrive==0.6.5)
RUN python3 -m venv --system-site-packages /opt/venv/cabot3

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

# install entrypoint
COPY docker-entrypoint.sh \
	launch.sh \
        launch-core.sh \
        launch-bag.sh \
	/

USER $USERNAME

ENV HOME /home/$USERNAME

WORKDIR $HOME/ros2_ws

ENTRYPOINT [ "/docker-entrypoint.sh" ]
CMD [ "/launch.sh" ]
