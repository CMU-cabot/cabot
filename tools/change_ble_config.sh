#!/bin/sh

# Copyright (c) 2020  Carnegie Mellon University
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

SUPERVISION_TIMEOUT=1000
MINIMUM_INTERVAL=6
MAXIMUM_INTERVAL=20

if [ -e /sys/kernel/debug/bluetooth/hci0 ]; then
    echo $SUPERVISION_TIMEOUT | sudo tee /sys/kernel/debug/bluetooth/hci0/supervision_timeout > /dev/null
    echo $MINIMUM_INTERVAL | sudo tee /sys/kernel/debug/bluetooth/hci0/conn_min_interval > /dev/null
    echo $MAXIMUM_INTERVAL | sudo tee /sys/kernel/debug/bluetooth/hci0/conn_max_interval > /dev/null
fi
if [ -e /sys/kernel/debug/bluetooth/hci1 ]; then
    echo $SUPERVISION_TIMEOUT | sudo tee /sys/kernel/debug/bluetooth/hci1/supervision_timeout > /dev/null
    echo $MINIMUM_INTERVAL | sudo tee /sys/kernel/debug/bluetooth/hci1/conn_min_interval > /dev/null
    echo $MAXIMUM_INTERVAL | sudo tee /sys/kernel/debug/bluetooth/hci1/conn_max_interval > /dev/null
fi
