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

pwd=`pwd`
scriptdir=`dirname $0`
cd $scriptdir
scriptdir=`pwd`

cd $scriptdir/../track_people_py/scripts

if [ ! -e "yolov4.cfg" ]; then
    echo "Downloading yolov4.cfg"
    wget https://raw.githubusercontent.com/AlexeyAB/darknet/yolov4/cfg/yolov4.cfg
else
    echo "You already have yolov4.cfg"
fi

if [ ! -e "yolov4.weights" ]; then
    echo "Downloading yolov4.weights"
    wget https://github.com/AlexeyAB/darknet/releases/download/darknet_yolo_v3_optimal/yolov4.weights
else
    echo "You already have yolov4.weights"
fi

if [ ! -e "coco.names" ]; then
    echo "Downloading coco.names"
    wget https://raw.githubusercontent.com/AlexeyAB/darknet/yolov4/cfg/coco.names
else
    echo "You already have coco.names"
fi