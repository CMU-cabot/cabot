#!/bin/bash

# Copyright (c) 2022 Carnegie Mellon University
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

sudo apt install -y software-properties-common && \
sudo add-apt-repository universe && \
sudo apt update && sudo apt install -y curl  && \
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg && \
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
    | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null && \
sudo apt update && sudo apt install -y ros-galactic-desktop

# workaround
# sudo dpkg -i --force-overwrite /var/cache/apt/archives/python3-catkin-pkg-modules_0.5.2-1_all.deb
# sudo dpkg -i --force-overwrite /var/cache/apt/archives/python3-rospkg-modules_1.4.0-1_all.deb
# sudo dpkg -i --force-overwrite /var/cache/apt/archives/python3-rosdistro-modules_0.9.0-1_all.deb
# sudo apt -f install

sudo apt install -y ros-dev-tools

echo "source /opt/ros/galactic/setup.bash" >> ~/.bashrc && \
source ~/.bashrc
