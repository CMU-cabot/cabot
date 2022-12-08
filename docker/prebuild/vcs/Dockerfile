ARG FROM_IMAGE
FROM $FROM_IMAGE

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
