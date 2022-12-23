#!/usr/bin/python
# -*- coding: utf-8 -*-

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

import os
import rospy
import re
import json
from std_msgs.msg import String

cellNumberRe = re.compile(r"^BSS\s+(?P<mac>.+)\(on\s(?P<interface>.+)\)*$")
regexps = [
    re.compile(r"^freq:\s(?P<frequency>.*)$"), #MHz
    re.compile(r"^signal:\s(?P<signal_level_dBm>.*)\sdBm$"),
    re.compile(r"^SSID:\s(?P<essid>.*)$"),
    re.compile(r"^last seen: (?P<last_seen>.*) ms ago$"),
]

def parse(content):
    cells = []
    lines = content.split('\n')
    for line in lines:
        line = line.strip()
        print(line)
        cellNumber = cellNumberRe.search(line)
        if cellNumber is not None:
            cells.append(cellNumber.groupdict())
            continue
        for expression in regexps:
            result = expression.search(line)
            if result is not None:
                cells[-1].update(result.groupdict())
                continue
    return cells

def convert(cells):
    machine_name = os.uname()[0] + "-" + os.uname()[1]
    now = rospy.get_time()
    json_object = {
        "type":"rssi",
        "phoneID":machine_name,
        "timestamp":now
    }
    data = []
    for cell in cells:
        ssid = cell["essid"] if "essid" in cell else ""
        mac = cell["mac"]
        id_str = ssid + "-" + mac
        rssi = cell["signal_level_dBm"]
        freq = float(cell["frequency"])*0.001
        last_seen_ms = int(cell["last_seen"])
        data.append({
            "type": "WiFi",
            "id" : id_str,
            "ssid": ssid,
            "mac": mac,
            "rssi" : float(rssi),
            "frequency_GHz": freq,
            "last_seen_ms": last_seen_ms
        })
    json_object["data"] = data
    return json_object


if __name__ == "__main__":

    rospy.init_node("wifi_scan_converter")
    pub = rospy.Publisher("/wireless/wifi", String, queue_size=100)

    def wifi_scan_str_callback(message):
        cells = parse(message.data)
        obj = convert(cells)
        print(obj)

        string = String()
        string.data = json.dumps(obj)
        pub.publish(string)

    sub = rospy.Subscriber("/wireless/wifi_iw_scan_str", String, wifi_scan_str_callback )

    rospy.spin()
