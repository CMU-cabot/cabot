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

rs_ids=()
for device in $(ls /sys/bus/usb/devices/*/product); do
    device_info=$( cat $device )

    if [[ $device_info == *"RealSense"* ]]; then
        device_id=$(echo $device | sed "s/\/sys\/bus\/usb\/devices\/\(.*\)\/product$/\1/")
        rs_ids=("${rs_ids[@]}" $device_id)
    fi
done

if [ ${#rs_ids[@]} = 0 ] ; then
    echo "No RealSense device"
else
    echo "Found RealSense devices : ${rs_ids[@]}"
fi

for rs_id in "${rs_ids[@]}"; do
    echo "Disconnect RealSense $rs_id"
    echo -n $rs_id > /sys/bus/usb/drivers/usb/unbind
    echo "Reconnect RealSense $rs_id"
    echo -n $rs_id > /sys/bus/usb/drivers/usb/bind
done
