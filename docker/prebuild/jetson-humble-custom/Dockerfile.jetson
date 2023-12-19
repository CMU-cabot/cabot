ARG FROM_IMAGE

FROM $FROM_IMAGE as cache

ENV ROS_DISTRO=humble

ENV CUSTOM_DIR=/opt/custom_ws

RUN mkdir -p $CUSTOM_DIR/src
COPY . $CUSTOM_DIR/src

WORKDIR $CUSTOM_DIR

RUN rosdep update && rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -r -y

RUN . /opt/ros/$ROS_DISTRO/install/setup.sh && \
	colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release  # --executor sequential

FROM $FROM_IMAGE as build

ENV CUSTOM_DIR=/opt/custom_ws

RUN mkdir -p $CUSTOM_DIR/install
COPY --from=cache $CUSTOM_DIR/install $CUSTOM_DIR/install
RUN sed -i "s:/ros/\$ROS_DISTRO/install/:/custom_ws/install/:" /ros_entrypoint.sh