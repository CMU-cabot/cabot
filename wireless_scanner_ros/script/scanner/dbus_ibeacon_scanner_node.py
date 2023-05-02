#!/usr/bin/env python3

###############################################################################
# Copyright (c) 2020, 2022  Carnegie Mellon University, IBM Corporation and others
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
###############################################################################
'''
This is an implementation to scan iBeacons by using bluez dbus API
'''
import json
import signal
import sys
import threading
import time
import traceback

from gi.repository import GLib

import dbus
import dbus.service
import dbus.mainloop.glib

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from diagnostic_updater import Updater, FunctionDiagnosticTask
from diagnostic_msgs.msg import DiagnosticStatus

BLUEZ_DEVICE1 = 'org.bluez.Device1'
RSSI_PROPERTY = 'RSSI'
MANUFACTURER_DATA_PROPERTY = 'ManufacturerData'
APPLE_COMPANY_ID = 0x004C
IBEACON_TYPE = b'\x02\x15'


def parse_props(props):
    """parse ibeacon from bluez device properties"""
    if RSSI_PROPERTY not in props:
        return
    if MANUFACTURER_DATA_PROPERTY not in props:
        return
    rssi = int(props[RSSI_PROPERTY])
    data = props[MANUFACTURER_DATA_PROPERTY]

    if APPLE_COMPANY_ID not in data:
        return
    payload = data[APPLE_COMPANY_ID]

    if payload[0:2] != IBEACON_TYPE:
        return

    uuid = "{}-{}-{}-{}-{}".format(payload[2:6].hex(),
                                   payload[6:8].hex(),
                                   payload[8:10].hex(),
                                   payload[10:12].hex(),
                                   payload[12:18].hex())
    major = int.from_bytes(payload[18:20], "big")
    minor = int.from_bytes(payload[20:22], "big")
    power = int.from_bytes(payload[22:23], "big")

    power = power if power < 128 else (power-256)

    result = {"uuid": uuid, "major": major, "minor": minor, "power": power, "rssi": rssi}
    result["scanner"] = "dbus"

    # publish as a ROS message
    beacon_scan_str_msg = String()
    beacon_scan_str_msg.data = json.dumps(result)
    pub.publish(beacon_scan_str_msg)
    node.get_logger().info("beacon updated", throttle_duration_sec=1)


def interfaces_added(_, kwargs):
    """callback for InterfacesAdded signal"""
    if BLUEZ_DEVICE1 not in kwargs:
        return
    parse_props(kwargs[BLUEZ_DEVICE1])


def properties_changed(path, props, _):
    """callback for PropertiesChanged signal"""
    if BLUEZ_DEVICE1 != path:
        return
    parse_props(props)


def check_status(stat):
    node.get_logger().info("check_status")
    try:
        powered = bluez_properties.Get("org.bluez.Adapter1", "Powered")
        discovering = bluez_properties.Get("org.bluez.Adapter1", "Discovering")

        if powered:
            if discovering:
                stat.summary(DiagnosticStatus.OK, "Bluetooth is on and discoverying beacons")
            else:
                stat.summary(DiagnosticStatus.WARN, "Bluetooth is on but not discoverying beacons")
        else:
            stat.summary(DiagnosticStatus.ERROR, "Bluetooth is off")
    except:  # noqa: E722
        stat.summary(DiagnosticStatus.ERROR, "Bluetooth service error")
    return stat


def shutdown_hook(signal_num, frame):
    loop.quit()
    print("shutdown_hook dbus_ibeacon_scanner_node.py")
    sys.exit(0)


signal.signal(signal.SIGINT, shutdown_hook)


discovery_started = False
discovery_start_time = None


def polling_bluez():
    global discovery_started, discovery_start_time, loop
    while not quit_flag:
        try:
            powered = bluez_properties.Get("org.bluez.Adapter1", "Powered")
            discovering = bluez_properties.Get("org.bluez.Adapter1", "Discovering")
            discovery_start_time = time.time()
            node.get_logger().info("power {}, discovering {}".format(powered, discovering))
            if powered and not discovering:
                try:
                    node.get_logger().info("SetDiscoveryFilter")
                    bluez_adapter.SetDiscoveryFilter(dbus.types.Dictionary({
                        # to show all changes
                        "DuplicateData": dbus.types.Boolean(True),
                        "RSSI": dbus.types.Int16(-127),
                        "Transport": dbus.types.String("le")
                    }))
                    node.get_logger().info("SetDiscovery")
                    bluez_adapter.StartDiscovery()
                    discovery_start_time = time.time()
                    node.get_logger().info("bluez discovery started")
                except:  # noqa: E722
                    node.get_logger().error(traceback.format_exc())
            elif not powered:
                node.get_logger().info("bluetooth is disabled")
            time.sleep(1.0)

            if discovery_start_time and time.time() - discovery_start_time > restart_interval:
                node.get_logger().info("Stop discovery intentionaly to prevend no scanning")
                bluez_adapter.StopDiscovery()
                discovery_started = False
        except:  # noqa: E722
            discovery_started = False
            node.get_logger().error(traceback.format_exc())
            break
    loop.quit()
    global polling_thread
    polling_thread = None


quit_flag = False


def sigint_handler(sig, frame):
    print("sigint_handler")
    global quit_flag
    if sig == signal.SIGINT:
        loop.quit()
        quit_flag = True
    else:
        print("Unexpected signal")


if __name__ == '__main__':
    signal.signal(signal.SIGINT, sigint_handler)
    rclpy.init()
    node = Node("dbus_ibeacon_scanner")
    adapter = node.declare_parameter("adapter", "hci0").value
    restart_interval = node.declare_parameter("restart_interval", 60).value

    pub = node.create_publisher(String, "wireless/beacon_scan_str", 10)
    polling_thread = None

    updater = Updater(node)
    updater.setHardwareID(adapter)
    updater.add(FunctionDiagnosticTask("Beacon Scanner", check_status))

    def spin():
        rclpy.spin(node)
    thread = threading.Thread(target=spin)
    thread.start()

    while not quit_flag:
        node.get_logger().info("start")
        dbus.mainloop.glib.DBusGMainLoop(set_as_default=True)

        node.get_logger().info("get system bus")
        system_bus = dbus.SystemBus()

        node.get_logger().info("get bluez objects")
        bluez_object = system_bus.get_object("org.bluez", "/org/bluez/" + adapter)
        bluez_adapter = dbus.Interface(bluez_object, "org.bluez.Adapter1")
        bluez_properties = dbus.Interface(bluez_adapter, 'org.freedesktop.DBus.Properties')

        node.get_logger().info("listen to signals")
        system_bus.add_signal_receiver(interfaces_added,
                                       dbus_interface="org.freedesktop.DBus.ObjectManager",
                                       signal_name="InterfacesAdded",
                                       byte_arrays=True)

        system_bus.add_signal_receiver(properties_changed,
                                       dbus_interface="org.freedesktop.DBus.Properties",
                                       signal_name="PropertiesChanged",
                                       arg0="org.bluez.Device1",
                                       byte_arrays=True)

        if not polling_thread:
            node.get_logger().info("starting thread")
            polling_thread = threading.Thread(target=polling_bluez)
            polling_thread.start()

        try:
            node.get_logger().info("loop")
            loop = GLib.MainLoop()
            loop.run()
        except:  # noqa: E722
            break
        node.get_logger().info("loop quit")
