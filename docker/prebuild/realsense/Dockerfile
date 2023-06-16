# Copyright (c) 2021, 2022  IBM Corporation and Carnegie Mellon University
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

FROM $FROM_IMAGE AS build
ARG UBUNTU_DISTRO=jammy
ENV DEBIAN_FRONTEND="noninteractive"

RUN apt update && \
    apt install -y --no-install-recommends \
    dirmngr \
    gpg-agent \
    software-properties-common \
    curl \
    && \
    apt clean && \
    rm -rf /var/lib/apt/lists/*

# install realsense
RUN mkdir -p /etc/apt/keyrings && \
    curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp | tee /etc/apt/keyrings/librealsense.pgp > /dev/null && \
    echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo `lsb_release -cs` main" | \
    tee /etc/apt/sources.list.d/librealsense.list && \
    apt-get update && \
    apt install -y --no-install-recommends \
    librealsense2 \
    librealsense2-gl \
#    librealsense2-net \  # avoid dependency error @ 2023.06.07, may not be used
    librealsense2-udev-rules \
    librealsense2-utils \
    librealsense2-dev \
    librealsense2-dbg \
    librealsense2-dkms \
    && \
    apt clean && \
    rm -rf /var/lib/apt/lists/*
