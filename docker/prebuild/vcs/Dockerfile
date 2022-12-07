ARG FROM_IMAGE
FROM $FROM_IMAGE

RUN apt update && apt install -q -y --no-install-recommends \
	python3-pip \
	git \
	&& \
	apt clean && \
	rm -rf /var/lib/apt/lists/*

RUN pip3 install --no-cache vcstool
