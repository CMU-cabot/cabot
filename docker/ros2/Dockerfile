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
ENV ROS_DISTRO=galactic

RUN apt update && \
    apt install -y --no-install-recommends \
    bc \
    libqt5svg5-dev \
    libzmq3-dev \
    libdw-dev \
    qtbase5-dev \
    python3-pykdl \
    ros-$ROS_DISTRO-rviz2 \
    ros-$ROS_DISTRO-diagnostic-updater \
    xterm \
    && \
    rm -rf /var/lib/apt/lists/*

# for debug
#RUN apt update && \
#    apt install -y --no-install-recommends \
#    gdb \
#    vim \
#    && \
#    rm -rf /var/lib/apt/lists/*

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
	/

USER $USERNAME

ENV HOME /home/$USERNAME

WORKDIR $HOME/ros2_ws

#ENTRYPOINT [ "docker-entrypoint.sh" ]
CMD [ "/launch.sh" ]
