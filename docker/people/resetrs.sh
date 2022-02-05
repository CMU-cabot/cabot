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

serial=$1

readarray -t ids < <(rs-enumerate-devices -S | sed -nE 's/.*usb.\/([0-9-]+).*/\1/p')
readarray -t serials < <(rs-enumerate-devices -S | sed -nE 's/^Intel RealSense ....\s+([0-9]+).*/\1/p')

END=${#ids[@]}
rs_ids=()
for ((i=0;i<END;i++)); do
    if [ -z $serial ] || [ ${serials[$i]} = $serial ]; then
	echo ${ids[$i]}, ${serials[$i]}
	rs_ids+=(${ids[$i]})
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
