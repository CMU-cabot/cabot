#ARG FROM_IMAGE=rosplanning/navigation2:master.release
ARG FROM_IMAGE=foxy-ros-desktop-nav2-focal

FROM $FROM_IMAGE
ENV ROS_DISTRO=foxy

RUN apt update && \
    apt install -y xterm vim \
	qtbase5-dev libqt5svg5-dev libzmq3-dev libdw-dev \
	bc gdb python3-pykdl
#	ros-$ROS_DISTRO-xacro \
#	ros-$ROS_DISTRO-rosbag2


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
	groot.sh \
	launch.sh \
	/

USER $USERNAME

ENV HOME /home/$USERNAME

WORKDIR $HOME/ros2_ws

#ENTRYPOINT [ "docker-entrypoint.sh" ]
CMD [ "/launch.sh" ]
