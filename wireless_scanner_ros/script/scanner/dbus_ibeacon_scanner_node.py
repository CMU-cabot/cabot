#!/usr/bin/env python

###############################################################################
# Copyright (c) 2020,2021  Carnegie Mellon University, IBM Corporation and others
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

from gi.repository import GLib

import dbus
import dbus.service
import dbus.mainloop.glib

import codecs
import json
import struct
import sys

import rospy
from std_msgs.msg import String

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

    if sys.version_info[0] >= 3:
        # Python 3
        uuid = "{}-{}-{}-{}-{}".format(payload[2:6].hex(),
                                       payload[6:8].hex(),
                                       payload[8:10].hex(),
                                       payload[10:12].hex(),
                                       payload[12:18].hex())
        major = int.from_bytes(payload[18:20], "big")
        minor = int.from_bytes(payload[20:22], "big")
        power = int.from_bytes(payload[22:23], "big")
    else:
        # Python 2.7
        uuid = "{}-{}-{}-{}-{}".format(codecs.encode(payload[2:6], "hex"),
                                       codecs.encode(payload[6:8], "hex"),
                                       codecs.encode(payload[8:10], "hex"),
                                       codecs.encode(payload[10:12], "hex"),
                                       codecs.encode(payload[12:18], "hex"))
        major = struct.unpack(">H", payload[18:20])[0] # ">H": big-endian & unsigned short
        minor = struct.unpack(">H", payload[20:22])[0] # ">H": big-endian & unsigned short
        power = struct.unpack(">B", payload[22:23])[0] # ">B": big-endian & unsigned char

    power = power if power < 128 else (power-256)

    result = {"uuid": uuid, "major": major, "minor": minor, "power": power, "rssi": rssi}
    result["scanner"] = "dbus"

    # publish as a ROS message
    beacon_scan_str_msg = String()
    beacon_scan_str_msg.data = json.dumps(result)
    pub.publish(beacon_scan_str_msg)

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


if __name__ == '__main__':
    rospy.init_node("dbus_ibeacon_scanner")
    adapter = rospy.get_param("~adapter","hci0")
    pub = rospy.Publisher("wireless/beacon_scan_str", String, queue_size=10)

    dbus.mainloop.glib.DBusGMainLoop(set_as_default=True)

    system_bus = dbus.SystemBus()

    bluez_object = system_bus.get_object("org.bluez", "/org/bluez/" + adapter)
    bluez_adapter = dbus.Interface(bluez_object, "org.bluez.Adapter1")

    system_bus.add_signal_receiver(interfaces_added,
                                   dbus_interface = "org.freedesktop.DBus.ObjectManager",
                                   signal_name = "InterfacesAdded",
                                   byte_arrays = True)

    system_bus.add_signal_receiver(properties_changed,
                                   dbus_interface = "org.freedesktop.DBus.Properties",
                                   signal_name = "PropertiesChanged",
                                   arg0 = "org.bluez.Device1",
                                   byte_arrays = True)

    bluez_adapter.SetDiscoveryFilter(dbus.types.Dictionary({
        # to show all changes
        "DuplicateData": dbus.types.Boolean(True),
        "RSSI": dbus.types.Int16(-127),
        "Transport": dbus.types.String("le")
    }))
    bluez_adapter.StartDiscovery()

    loop = GLib.MainLoop()
    loop.run()
