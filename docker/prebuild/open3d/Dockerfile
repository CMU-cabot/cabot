# Copyright (c) 2021  IBM Corporation
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

ARG OPEN3D_VERSION=0.15.1

RUN apt update && \
	apt install -y --no-install-recommends \
	git \
	python3 \
	python3-dev \
	xorg-dev \
        libglu1-mesa-dev \
	libc++-dev \
	&& \
	apt clean && \
	rm -rf /var/lib/apt/lists/*

# install open3d for c++
RUN cd /tmp && \
#	wget https://github.com/isl-org/Open3D/releases/download/v${OPEN3D_VERSION}/open3d-devel-linux-x86_64-cxx11-abi-${OPEN3D_VERSION}.tar.xz && \
	wget https://github.com/isl-org/Open3D/releases/download/v${OPEN3D_VERSION}/open3d-devel-linux-x86_64-cxx11-abi-cuda-${OPEN3D_VERSION}.tar.xz && \
#	tar xf open3d-devel-linux-x86_64-cxx11-abi-${OPEN3D_VERSION}.tar.xz && \
        tar xf open3d-devel-linux-x86_64-cxx11-abi-cuda-${OPEN3D_VERSION}.tar.xz && \
#	cp -r open3d-devel-linux-x86_64-cxx11-abi-${OPEN3D_VERSION}/* /usr/local && \
	cp -r open3d-devel-linux-x86_64-cxx11-abi-cuda-${OPEN3D_VERSION}/* /usr/local && \
	rm -rf *

# TODO: remove open3d python
# install python open3d
RUN apt update && \
	apt install -y --no-install-recommends \
	python3-pip \
	&& \
	apt clean && \
	rm -rf /var/lib/apt/lists/* && \
	pip3 install --no-cache-dir open3d

# need to run build, install, and delete to reduce image size
# RUN git clone --recursive https://github.com/intel-isl/Open3D && \
# 	cd Open3D && \
# 	git checkout v0.14.1 && \
# 	git submodule update --init --recursive && \
# 	mkdir build && \
# 	cd build && \
# 	cmake \
# 	-DCMAKE_BUILD_TYPE=Release \
# 	-DBUILD_EXAMPLES=OFF \
# 	-DBUILD_CUDA_MODULE=OFF \
# 	-DBUILD_GUI=OFF \
# 	-DBUILD_TENSORFLOW_OPS=OFF \
# 	-DBUILD_PYTORCH_OPS=OFF \
# 	-DBUILD_UNIT_TESTS=OFF \
# 	-DPYTHON_EXECUTABLE=$(which python3) \
# # to avoid error when linking with cpp programs
# 	-DBUILD_SHARED_LIBS=ON \
# 	-DGLIBCXX_USE_CXX11_ABI=ON \
# 	.. && \
# 	make -j$(nproc) && \
# 	make install && \
# 	make install-pip-package && \
# 	make python-package && \
# 	make pip-package && \
#         cd ../../ && \
# 	rm -rf Open3D

