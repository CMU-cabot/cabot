#!/bin/bash

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

args=("$@")

WS=$HOME/people_ws

if [ "$1" == "build" ]; then
    cd $WS
    catkin_make

    cd $WS/src/track_people_py/scripts/darknet/
    mkdir build_release
    cd build_release
    cmake ..
    cmake --build . --target install --parallel 8

    cd $WS/src/queue_utils_py
    pip3 install -e .
    exit
else
    echo "Skip building workscape"
fi

source devel/setup.bash
# set environment variable to load dynamic library from python
export DARKNET_PATH=$WS/src/track_people_py/scripts/darknet

# reset RealSense port
sudo /resetrs.sh


cd $WS/src/cabot_people/script
exec ./cabot_people.sh ${args[@]}
