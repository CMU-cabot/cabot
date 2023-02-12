#!/usr/bin/python3
# -*- coding: utf-8 -*-

# Copyright (c) 2021, 2022  IBM Corporation
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
import time
import json
import signal
import sys

import rclpy
from rclpy.node import Node

import numpy as np
from std_msgs.msg import String
from diagnostic_updater import Updater, FunctionDiagnosticTask
from diagnostic_msgs.msg import DiagnosticStatus

# deprecated function


def convert(now, beacon):
    machine_name = os.uname()[0] + "-" + os.uname()[1]
    json_object = {
        "type": "rssi",
        "phoneID": machine_name,
        "timestamp": now
    }

    ibeacon = beacon["iBeacon"]
    data = [
        {
            "type": "iBeacon",
            "id": ibeacon["uuid"]+"-"+str(ibeacon["major"])+"-"+str(ibeacon["minor"]),
            "rssi": beacon["rssi"]
        }
    ]
    json_object["data"] = data
    return json_object


def convert_element(now, beacon):
    # message.data format
    # {
    #   "id": id
    #   "address": address,
    #   "localName": localName,
    #   "txPowerLevel": txPowerLevel,
    #   "rssi": rssi,
    #   "beaconType": beaconType,
    #   "iBeacon":{
    #              "uuid": uuid,
    #              "major": major,
    #              "minor": minor,
    #              "txPower": power
    #              }
    #    }
    if "iBeacon" in beacon:
        ibeacon = beacon["iBeacon"]
        elem = {
            "type": "iBeacon",
            "id": ibeacon["uuid"]+"-"+str(ibeacon["major"])+"-"+str(ibeacon["minor"]),
            "rssi": beacon["rssi"],
            "timestamp": now
        }
        return elem
    else:
        # it could find eddystone
        return None


def convert_dbus_element(now, beacon):
    # message.data format
    # {
    #    "scanner": "dbus", # fixed value
    #    "uuid": uuid,
    #    "major": major,
    #    "minor": minor,
    #    "power": power,
    #    "rssi": rssi
    #   }
    uuid = beacon["uuid"].upper()  # convert to upper case for compatibility
    elem = {
        "type": "iBeacon",
        "id": uuid+"-"+str(beacon["major"])+"-"+str(beacon["minor"]),
        "rssi": beacon["rssi"],
        "timestamp": now
    }
    return elem


def average_elements(elements):
    dic = {}
    for b in elements:
        if b["id"] in dic.keys():
            dic[b["id"]]["rssi"].append(b["rssi"])
            dic[b["id"]]["timestamp"].append(b["timestamp"])
        else:
            dic[b["id"]] = {
                "rssi": [b["rssi"]],
                "timestamp": [b["timestamp"]]
            }
    results = []
    for key in dic.keys():
        elem = {
            "type": "iBeacon",
            "id": key,
            "rssi": np.mean(dic[key]["rssi"]),
            "timestamp": np.mean(dic[key]["timestamp"]),
            "count": len(dic[key]["rssi"])
        }
        results.append(elem)
    return results


class BeaconAccumulator:
    def __init__(self):
        self.beacons = []

    def put_beacon(self, now, beacon):
        elem = None

        # switch converter based on input
        if "scanner" in beacon:
            if beacon["scanner"] == "dbus":  # dbus_ibeacon_scanner.py
                elem = convert_dbus_element(now, beacon)
        else:  # ble_scanner.js
            elem = convert_element(now, beacon)

        if elem is None:
            return

        self.beacons.append(elem)

    def put_dbus_beacon(self, now, beacon):
        elem = convert_dbus_element(now, beacon)
        if elem is None:
            return
        self.beacons.append(elem)

    def get_average(self):
        now = time.time()
        machine_name = os.uname()[0] + "-" + os.uname()[1]
        data = average_elements(self.beacons)
        json_obj = {
            "type": "rssi",
            "phoneID": machine_name,
            "timestamp": now,
            "data": data
        }

        return json_obj

    def clear(self):
        self.beacons = []


is_active = False
last_active = None
beacon_num = 0


def check_status(stat):
    global is_active
    if last_active is None:
        is_active = False
    else:
        if (time.time() - last_active) > 3:
            is_active = False
    if is_active:
        if beacon_num == 0:
            stat.summary(DiagnosticStatus.OK, "No beacon is found")
        elif beacon_num == 1:
            stat.summary(DiagnosticStatus.OK, "1 beacon is found")
        else:
            stat.summary(DiagnosticStatus.OK, "{} beacons are found".format(beacon_num))
    else:
        stat.summary(DiagnosticStatus.WARN, "No beacon is found")
    return stat


def shutdown_hook(signal_num, frame):
    print("shutdown_hook ble_scan_converter.py")
    sys.exit(0)


signal.signal(signal.SIGINT, shutdown_hook)


if __name__ == "__main__":
    rclpy.init()
    node = Node("ble_scan_converter")

    rate = node.declare_parameter("rate", 1.0).value  # default = 1.0 Hz

    pub = node.create_publisher(String, "/wireless/beacons", 100)

    updater = Updater(node)
    updater.setHardwareID("scan_converter")
    updater.add(FunctionDiagnosticTask("Beacon Converter", check_status))

    accum = BeaconAccumulator()

    def ble_scan_str_callback(message):
        now = time.time()
        beacon = json.loads(message.data)
        accum.put_beacon(now, beacon)
        global is_active, last_active
        is_active = True
        last_active = now

    sub = node.create_subscription(String, "/wireless/beacon_scan_str", ble_scan_str_callback, 10)

    def timer_callback():
        json_obj = accum.get_average()
        accum.clear()

        beacon_num = len(json_obj["data"])
        if beacon_num > 0:
            string = json.dumps(json_obj)
            msg = String()
            msg.data = string
            pub.publish(msg)

    timer = node.create_timer(rate, timer_callback)
    try:
        rclpy.spin(node)
    except:  # noqa: E722
        pass
