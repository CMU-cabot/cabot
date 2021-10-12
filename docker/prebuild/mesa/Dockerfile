ARG FROM_IMAGE=ubuntu
FROM $FROM_IMAGE
ENV DEBIAN_FRONTEND="noninteractive"

RUN apt update && \
	apt install -y \
	mesa-utils \
	xserver-xorg-video-all \
	&& \
	rm -rf /var/lib/apt/lists/*

