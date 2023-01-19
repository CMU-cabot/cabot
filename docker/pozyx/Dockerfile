from ros:noetic


RUN apt update && apt install -y \
	python3-pip

RUN pip3 install \
	paho-mqtt \
	pyyaml \
	rospkg

RUN apt install -y \
        ros-noetic-tf


COPY uwb_positioning.py uwb_positioning.py

