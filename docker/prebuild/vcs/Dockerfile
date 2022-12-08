ARG FROM_IMAGE
FROM $FROM_IMAGE

RUN apt update && apt install -q -y --no-install-recommends \
	python3-pip \
	python3-rosdep \
	git \
	&& \
	apt clean && \
	rm -rf /var/lib/apt/lists/* && \
	rosdep init

RUN pip3 install --no-cache vcstool
