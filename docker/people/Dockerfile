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
ARG ROS_DISTRO=noetic \
    UBUNTU_DISTRO=focal \
    GAZEBOV=11 \
    TZ="Etc/UTC" \
# CMake 3.19+ is required to build Open3D
    CMAKE_V=3.19.3 \
    OPENCV_V=4.5.4 \
    COMP_CAP=6.1,7.5,8.6 \
    LIBREALSENSE_V=2.50.0-0~realsense0.6128


ENV ROS_DISTRO=$ROS_DISTRO \
    TZ=$TZ

RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

#update gazebo to latest
# for kinetic
RUN apt update && \
	apt install -y --no-install-recommends \
	git \
        libfreetype6-dev \
        libglu1-mesa-dev \
	python-tk \
	python3-pip \
	python3-vcstool \
	rsync \
	software-properties-common \
	unzip \
	usbutils \
	wget \
	xorg-dev \
	&& \
	apt clean && \
	rm -rf /var/lib/apt/lists/*

# install required packages
RUN sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable $UBUNTU_DISTRO main" > /etc/apt/sources.list.d/gazebo-stable.list' && \
    wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add - && \
    apt update && \
    apt install -y --no-install-recommends \
    gazebo$GAZEBOV \
    libgazebo$GAZEBOV-dev \
    ros-$ROS_DISTRO-compressed-image-transport \
    ros-$ROS_DISTRO-diagnostic-updater \
    ros-$ROS_DISTRO-gazebo-msgs \
    ros-$ROS_DISTRO-move-base-msgs \
    ros-$ROS_DISTRO-people-msgs \
    ros-$ROS_DISTRO-rviz \
    ros-$ROS_DISTRO-tf2-geometry-msgs \
    && \
    apt clean && \
    rm -rf /var/lib/apt/lists/*

# install cmake
RUN wget https://github.com/Kitware/CMake/releases/download/v${CMAKE_V}/cmake-${CMAKE_V}-linux-x86_64.sh && \
    sudo sh cmake-${CMAKE_V}-linux-x86_64.sh --skip-license --prefix=/usr && \
    rm cmake-${CMAKE_V}-linux-x86_64.sh

# install opencv        
RUN mkdir -p src/opencv/build && cd src/opencv && \
	wget https://github.com/opencv/opencv/archive/refs/tags/${OPENCV_V}.zip && \
	unzip ${OPENCV_V}.zip && rm ${OPENCV_V}.zip && \
	wget https://github.com/opencv/opencv_contrib/archive/refs/tags/${OPENCV_V}.zip && \
	unzip ${OPENCV_V}.zip && rm ${OPENCV_V}.zip && \
	cd build && \
	cmake -D CMAKE_BUILD_TYPE=RELEASE \
	-D CMAKE_INSTALL_PREFIX=/usr/local \
	-D INSTALL_PYTHON_EXAMPLES=OFF \
	-D INSTALL_C_EXAMPLES=OFF \
	-D WITH_TBB=ON \
	-D WITH_CUDA=ON \
	-D WITH_CUDNN=ON \
	-D OPENCV_DNN_CUDA=ON \
	-D CUDA_ARCH_BIN=${COMP_CAP} \
	-D BUILD_opencv_cudacodec=OFF \
	-D ENABLE_FAST_MATH=1 \
	-D CUDA_FAST_MATH=1 \
	-D WITH_CUBLAS=1 \
	-D WITH_V4L=ON \
	-D WITH_QT=ON \
	-D WITH_OPENGL=ON \
	-D WITH_GSTREAMER=ON \
	-D WITH_FFMPEG=ON \
	-D OPENCV_GENERATE_PKGCONFIG=ON \
	-D OPENCV_PC_FILE_NAME=opencv4.pc \
	-D OPENCV_ENABLE_NONFREE=OFF \
	-D OPENCV_EXTRA_MODULES_PATH=../opencv_contrib-${OPENCV_V}/modules \
	-D PYTHON_DEFAULT_EXECUTABLE=$(which python3) \
	-D BUILD_EXAMPLES=OFF \
	../opencv-${OPENCV_V} && \
	make -j$(nproc) && \
	make install && \
	ldconfig && \
	cd ../../ && \
	rm -rf opencv

COPY requirements.txt /
RUN pip3 install --no-cache-dir \
        -r /requirements.txt

# need to run build, install, and delete to reduce image size
RUN git clone --recursive https://github.com/intel-isl/Open3D && \
	cd Open3D && \
	git checkout v0.14.1 && \
	git submodule update --init --recursive && \
	mkdir build && \
	cd build && \
	cmake \
	-DCMAKE_BUILD_TYPE=Release \
	-DBUILD_EXAMPLES=OFF \
	-DBUILD_CUDA_MODULE=OFF \
	-DBUILD_GUI=OFF \
	-DBUILD_TENSORFLOW_OPS=OFF \
	-DBUILD_PYTORCH_OPS=OFF \
	-DBUILD_UNIT_TESTS=OFF \
	-DPYTHON_EXECUTABLE=$(which python3) \
# to avoid error when linking with cpp programs
	-DBUILD_SHARED_LIBS=ON \
	-DGLIBCXX_USE_CXX11_ABI=ON \
	.. && \
	make -j$(nproc) && \
	make install && \
        cd ../../ && \
	rm -rf Open3D

RUN apt-key adv --keyserver keys.gnupg.net --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || \
	apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE && \
	add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $UBUNTU_DISTRO main" -u && \
# temporal fix for build issue
# https://github.com/IntelRealSense/librealsense/issues/9607
        apt install -y --no-install-recommends \
	librealsense2=$LIBREALSENSE_V \
	librealsense2-gl=$LIBREALSENSE_V \
	librealsense2-net=$LIBREALSENSE_V \
	librealsense2-udev-rules=$LIBREALSENSE_V \
	librealsense2-utils=$LIBREALSENSE_V \
	librealsense2-dev=$LIBREALSENSE_V \
	librealsense2-dbg=$LIBREALSENSE_V && \
        apt install -y --no-install-recommends \
	librealsense2-dkms && \
	apt clean && \
	rm -rf /var/lib/apt/lists/*

RUN apt update && \
	apt-get install -y --no-install-recommends \
	libogg-dev \
        libtheora-dev \
	ros-$ROS_DISTRO-diagnostic-updater \
	&& \
	apt-get clean && \
	rm -rf /var/lib/apt/lists/*

COPY ./launch.sh ./resetusb.sh ./resetrs.sh /


# setup for display
ENV USERNAME developer

### replace 1000 with your user/group id
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

USER $USERNAME

ENV HOME /home/$USERNAME
WORKDIR $HOME/people_ws
