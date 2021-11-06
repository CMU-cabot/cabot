#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Copyright (c) 2021  IBM Corporation, Carnegie Mellon University, and others
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

import json
import argparse

import rospy
import tf2_ros
import tf_conversions
from std_msgs.msg import String
from sensor_msgs.msg import Image, PointCloud2

class ESP32WiFiScanConverter:
    def __init__(self):
        pass

    def convert_str(self, string):
        data = string.split(",")

        if len(data) != 6:
            rospy.logerr("invalid wifi_scan_str: len(data)="+str(len(data)))
            return

        bssid = data[0] # mac
        ssid = data[1]
        channel = data[2]
        rssi = int(data[3])
        seconds = int(data[4])
        nanoseconds = int(data[5])

        ts = float(seconds) + float(nanoseconds) * 1.0e-9

        id_str = ssid + "-" + bssid
        json_object = {
            "type": "rssi",
            "phoneID": "ESP32",
            "timestamp": ts,
            "data": [
                {
                    "type": "WiFi",
                    "id": id_str, 
                    "rssi": rssi,
                    "timestamp": ts,
                    "ssid": ssid, #(optional)
                    "mac": bssid, #(optional)
                    "channel": channel, #(optional)
                }
            ]
        }

        return json_object

    def convert(self, msg):
        string = msg.data
        return self.convert_str(string)

    def wifi_scan_str_callback(self, msg):
        json_object = self.convert(msg)
        print(json_object)

class ESP32WiFiScanAccumulator:
    def __init__(self, interval = 1.0, buffer_interval = 10.0):
        self.data_list = []
        self.interval = interval
        self.buffer_interval = buffer_interval

    def push(self, data):
        ts = data["timestamp"]
        # remove old scans
        self.data_list = [elem for elem in self.data_list if ts - elem["timestamp"] < self.buffer_interval]
        
        self.data_list.append(data)

    def get_latest_scans(self, timestamp):
        latest_scans = [elem for elem in self.data_list if timestamp - elem["timestamp"] < self.interval]

        latest_scan_obj = {
            "type": "rssi",
            "phoneID": "ESP32",
            "timestamp": timestamp,
        }
        data = []
        for scan in latest_scans:
            data.extend(scan["data"])

        latest_scan_obj["data"] = data
        return latest_scan_obj
        
def main():
    rospy.init_node('esp32_wifi_scan_converter')

    mapper = ESP32WiFiScanConverter()

    points2_sub = rospy.Subscriber("/esp32/wifi_scan_str", String, mapper.wifi_scan_str_callback)

    rospy.spin()

if __name__ == "__main__":
    main()
