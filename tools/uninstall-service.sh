#!/bin/bash

# Copyright (c) 2022  Carnegie Mellon University
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

if [[ $(id -u) -eq 0 ]]; then
    echo "please do not run as root: $0"
    exit
fi

scriptdir=$(dirname "$0")
cd "$scriptdir" || exit
scriptdir=$(pwd)

cd "$scriptdir/../" || exit
projectdir=$(pwd)
project=$(basename "$projectdir")

## uninstall cabot.service
INSTALL_DIR=$HOME/.config/systemd/user
systemctl --user disable cabot
rm "$INSTALL_DIR/cabot.service"

## uninstall ble-config.service
SYS_INSTALL_DIR=/etc/systemd/system
sudo systemctl disable cabot-config
sudo rm "$SYS_INSTALL_DIR/cabot-config.service"

## uninstall nvidia-smi sudo pliviledge
USERNAME=$(id -un)
sudo rm "/etc/sudoers.d/$USERNAME"

## remove symlink
sudo rm "/opt/$project"
sudo rm /opt/cabot
