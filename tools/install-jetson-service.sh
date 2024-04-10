#!/bin/bash

# Copyright (c) 2023  Carnegie Mellon University, IBM Corporation, and others
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

if [ $(id -u) -eq 0 ]; then
    echo "please do not run as root: $0"
    exit
fi

pwd=$(pwd)
scriptdir=$(dirname $0)
cd $scriptdir
scriptdir=$(pwd)

cd $scriptdir/../
projectdir=$HOME/cabot

mkdir -p $projectdir/tools
cp $scriptdir/cabot_jetson_config.sh $projectdir/tools/
cp $scriptdir/change_dds_settings.sh $projectdir/tools/
sudo ln -sf $projectdir /opt/cabot

## install cabot-jetson-config.service
SYS_INSTALL_DIR=/etc/systemd/system
sudo cp $scriptdir/config/cabot-jetson-config.service $SYS_INSTALL_DIR
sudo systemctl daemon-reload
sudo systemctl enable cabot-jetson-config --now
