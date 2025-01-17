ARG BASE_IMAGE=cabot-base
ARG FROM_IMAGE=cmucal/${BASE_IMAGE}:humble-desktop-custom-mesa

# build underlay
FROM ${FROM_IMAGE} AS cache

RUN apt update && apt install -y \
    nlohmann-json3-dev \
    && \
    rm -rf /var/lib/apt/lists/*

ENV UNDERLAY_WS=/opt/underlay_ws

RUN mkdir -p $UNDERLAY_WS/src
COPY --from=cabot_src ./cabot-drivers/docker/driver/ros_odrive $UNDERLAY_WS/src/ros_odrive

WORKDIR $UNDERLAY_WS

# copy package.xml files with parent directories for cache
RUN find . -name "src" -type d \
      -mindepth 1 -maxdepth 2 -printf '%P\n' \
      | xargs -I % mkdir -p /tmp/opt/% && \
    find . -name "package.xml" \
      | xargs cp --parents -t /tmp/opt && \
    find . -name "COLCON_IGNORE" \
      | xargs cp --parents -t /tmp/opt || true

RUN apt update && rosdep update && rosdep install -q -y \
	--from-paths src \
	--ignore-src

RUN . /opt/custom_ws/install/setup.sh && \
	colcon build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo


FROM ${FROM_IMAGE} AS build
ENV ROS_DISTRO=humble
ARG TZ=UTC
ENV TZ=$TZ
RUN sudo ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ | sudo tee /etc/timezone

ARG ROS_DISTRO

ENV UNDERLAY_WS=/opt/underlay_ws

WORKDIR $UNDERLAY_WS

COPY --from=cache $UNDERLAY_WS/install $UNDERLAY_WS/install
# copy package.xml files from cache
COPY --from=cache /tmp/opt/src $UNDERLAY_WS/src
RUN sed -i 's:custom_ws:underlay_ws:' /ros_entrypoint.sh


RUN apt update && \
    apt install -y --no-install-recommends \
    ros-$ROS_DISTRO-dwb-msgs \
    ros-$ROS_DISTRO-rosbridge-msgs \
    ros-$ROS_DISTRO-nav2-msgs \
    nlohmann-json3-dev \
    && \
    rm -rf /var/lib/apt/lists/*

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
# add user to video group to fix NvRmMemInitNvmap error on Jetson
# https://forums.developer.nvidia.com/t/jetpack-5-0-dp-nvrmmeminitnvmap-failed-with-permission-denied/219804
RUN usermod -aG video $USERNAME

USER $USERNAME

ENV HOME /home/$USERNAME

WORKDIR $HOME/bag_ws

RUN mkdir -p $HOME/bag_ws/src && \
    mkdir -p $HOME/bag_ws/script

COPY --chown=$USERNAME:$USERNAME --from=cabot_src ./cabot-navigation/docker/ros2/launch.sh /

COPY --chown=$USERNAME:$USERNAME --from=cabot_src ./cabot-common/cabot_msgs /home/developer/bag_ws/src/cabot_msgs
COPY --chown=$USERNAME:$USERNAME --from=cabot_src ./cabot-common/cabot_common /home/developer/bag_ws/src/cabot_common
COPY --chown=$USERNAME:$USERNAME --from=cabot_src ./cabot_debug /home/developer/bag_ws/src/cabot_debug
COPY --chown=$USERNAME:$USERNAME --from=cabot_src ./cabot-drivers/motor_controller /home/developer/bag_ws/src/motor_controller
COPY --chown=$USERNAME:$USERNAME --from=cabot_src ./cabot-drivers/docker/driver/ros_odrive/odrive_base /home/developer/bag_ws/src/ros_odrive/odrive_base
COPY --chown=$USERNAME:$USERNAME --from=cabot_src ./cabot-drivers/docker/driver/ros_odrive/odrive_node /home/developer/bag_ws/src/ros_odrive/odrive_node
COPY --chown=$USERNAME:$USERNAME --from=cabot_src ./cabot-drivers/power_controller/power_controller_msgs /home/developer/bag_ws/src/power_controller/power_controller_msgs
COPY --chown=$USERNAME:$USERNAME --from=cabot_src ./cabot-navigation/script /home/developer/bag_ws/script
COPY --chown=$USERNAME:$USERNAME --from=cabot_src ./cabot-navigation/mf_localization_msgs /home/developer/bag_ws/src/mf_localization_msgs
COPY --chown=$USERNAME:$USERNAME --from=cabot_src ./cabot-navigation/mf_localization_rviz /home/developer/bag_ws/src/mf_localization_rviz
COPY --chown=$USERNAME:$USERNAME --from=cabot_src ./cabot-navigation/queue_msgs /home/developer/bag_ws/src/queue_msgs
COPY --chown=$USERNAME:$USERNAME --from=cabot_src ./cabot-people/track_people_msgs /home/developer/bag_ws/src/track_people_msgs
COPY --chown=$USERNAME:$USERNAME --from=cabot_src ./cabot-description/cabot_description /home/developer/bag_ws/src/cabot_description
COPY --chown=$USERNAME:$USERNAME --from=cabot_src ./cabot-navigation/docker/localization/ublox/ublox_msgs /home/developer/bag_ws/src/ublox_msgs
COPY --chown=$USERNAME:$USERNAME --from=cabot_src ./cabot-navigation/docker/localization/ublox/ublox_serialization /home/developer/bag_ws/src/ublox_serialization
COPY --chown=$USERNAME:$USERNAME --from=cabot_src ./cabot-navigation/docker/localization/cartographer_ros/cartographer_ros_msgs /home/developer/bag_ws/src/cartographer_ros_msgs
COPY --chown=$USERNAME:$USERNAME --from=cabot_src ./cabot-common/docker/humble-custom/velodyne/velodyne_msgs /home/developer/bag_ws/src/velodyne_msgs
COPY --chown=$USERNAME:$USERNAME --from=cabot_src ./cabot-navigation/pedestrian_plugin_msgs /home/developer/bag_ws/src/pedestrian_plugin_msgs
COPY --chown=$USERNAME:$USERNAME --from=cabot_src ./host_ws/src/tf_bag /home/developer/bag_ws/src/tf_bag

RUN /ros_entrypoint.sh /launch.sh build

FROM ${FROM_IMAGE} AS final
ENV ROS_DISTRO=humble
ARG TZ=UTC
ENV TZ=$TZ
RUN sudo ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ | sudo tee /etc/timezone

ENV UNDERLAY_WS=/opt/underlay_ws

WORKDIR $UNDERLAY_WS

COPY --from=cache $UNDERLAY_WS/install $UNDERLAY_WS/install
# copy package.xml files from cache
COPY --from=cache /tmp/opt/src $UNDERLAY_WS/src
RUN sed -i 's:custom_ws:underlay_ws:' /ros_entrypoint.sh

ARG ROS_DISTRO

RUN apt update && \
    apt install -y --no-install-recommends \
    bc \
    gosu \
    ros-$ROS_DISTRO-dwb-msgs \
    ros-$ROS_DISTRO-rosbridge-msgs \
    ros-$ROS_DISTRO-nav2-lifecycle-manager \
    ros-$ROS_DISTRO-nav2-map-server \
    ros-$ROS_DISTRO-nav2-msgs \
    nlohmann-json3-dev \
    && \
    rm -rf /var/lib/apt/lists/*

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
# add user to video group to fix NvRmMemInitNvmap error on Jetson
# https://forums.developer.nvidia.com/t/jetpack-5-0-dp-nvrmmeminitnvmap-failed-with-permission-denied/219804
RUN usermod -aG video $USERNAME

COPY ros_entrypoint.sh /ros_entrypoint.sh
RUN sed -i 's:custom_ws:underlay_ws:' /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh

ENV HOME /home/$USERNAME

WORKDIR $HOME/bag_ws
RUN chown $USERNAME:$USERNAME $HOME/bag_ws

COPY --chown=$USERNAME:$USERNAME --from=cabot_src ./cabot-navigation/docker/ros2/launch.sh /

COPY --chown=$USERNAME:$USERNAME --from=build $HOME/bag_ws/install $HOME/bag_ws/install
COPY --chown=$USERNAME:$USERNAME --from=cabot_src ./cabot-navigation/script /home/developer/bag_ws/script

CMD [ "/launch.sh record" ]
