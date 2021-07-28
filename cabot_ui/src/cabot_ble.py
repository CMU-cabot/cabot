#!/usr/bin/env python

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

import traceback
import binascii
from collections import deque
import array
import pygatt
import rospy
import time
import sys
import json
import threading
import std_msgs.msg
import cabot_msgs.srv
import faceapp_msgs.srv
from cabot import util
from cabot.event import BaseEvent
from cabot_ui.event import NavigationEvent


from pygatt.util import uuid16_to_uuid
from uuid import UUID
from pygatt.backends import BLEBackend, Characteristic, BLEAddressType

import gatt
from gi.repository import GObject
import dbus

### Debug
if False:
    import logging
    from logging import StreamHandler, Formatter

    stream_handler = StreamHandler()
    handler_format = Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
    stream_handler.setFormatter(hundler_format)
    stream_handler.setLevel(logging.DEBUG)

    for key in logging.Logger.manager.loggerDict:
        logger = logging.Logger.manager.loggerDict[key]
        try:
            logger.setLevel(logging.DEBUG)
            logger.addHandler(stream_handler)
        except:
            pass


class CaBotBLE:
    UUID_FORMAT = "35CE{0:04X}-5E89-4C0D-A3F6-8A6A507C1BF1"

    def __init__(self, address, mgr):
        self.address = address
        self.mgr = mgr
        self.summon_uuid = UUID(CaBotBLE.UUID_FORMAT.format(0x09))
        self.dest_uuid = UUID(CaBotBLE.UUID_FORMAT.format(0x10))
        self.find_person_ready_uuid = UUID(CaBotBLE.UUID_FORMAT.format(0x11))
        self.find_person_uuid = UUID(CaBotBLE.UUID_FORMAT.format(0x12))
        self.speak_uuid = UUID(CaBotBLE.UUID_FORMAT.format(0x200))
        self.navi_uuid = UUID(CaBotBLE.UUID_FORMAT.format(0x300))
        self.content_uuid = UUID(CaBotBLE.UUID_FORMAT.format(0x400))
        self.sound_uuid = UUID(CaBotBLE.UUID_FORMAT.format(0x500))
        self.heartbeat_uuid = UUID(CaBotBLE.UUID_FORMAT.format(0x9999))
        self.data_buffer = {}
        self.eventPub = rospy.Publisher('/cabot/event', std_msgs.msg.String, queue_size=1)
        rospy.Subscriber("/cabot/event", std_msgs.msg.String, self._event_callback, None)


        self.adapter = pygatt.GATTToolBackend()
        self.target = None
        self.last_heartbeat = time.time()

        self.find_person_proxy = None
        self.last_check_find_person = rospy.Time(0)
        self.find_person_ready = False

        # speak
        #rospy.Service("/speak", cabot_msgs.srv.Speak, self.handleSpeak)
        self.queue = deque([])
        self.alive = False
        self.ready = False

    def start(self):
        self.alive = True
        self.start_time = time.time()
        try:
            while time.time() - self.start_time < 60*10 and self.alive:
                self.adapter.start(reset_on_start=False)
                target = None

                try:
                    rospy.loginfo("trying to connect to %s" % (self.address))
                    target = self.adapter.connect(self.address, timeout=15, address_type=pygatt.BLEAddressType.random)
                    target.exchange_mtu(64)
                except pygatt.exceptions.NotConnectedError:
                    rospy.logerr("device not connected %s" % (self.address))
                except pygatt.exceptions.NotificationTimeout:
                    rospy.logerr("setting exchange_mtu failed %s" % (self.address))
                    target = None

                if target is not None:
                    try:
                        self.target = target

                        # TODO: restore find person function
                        #try:
                        #    self.target.subscribe(self.find_person_uuid, self.find_person_callback, indication=False)
                        #    rospy.loginfo("subscribed to find_person")
                        #except:
                        #    rospy.loginfo("could not connect to find_person")
                        #    rospy.logerr(traceback.format_exc())

                        try:
                            self.target.subscribe(self.dest_uuid, self.destination_callback, indication=False)
                            rospy.loginfo("subscribed to destination %s" % (self.address))
                        except:
                            rospy.loginfo("could not connect to destination %s" % (self.address))
                            return

                        try:
                            self.target.subscribe(self.summon_uuid, self.summons_callback, indication=False)
                            rospy.loginfo("subscribed to summons %s" % (self.address))
                        except:
                            rospy.loginfo("could not connect to destination %s" % (self.address))
                            return

                        try:
                            self.target.subscribe(self.heartbeat_uuid, self.heartbeat_callback, indication=False)
                            rospy.loginfo("subscribed to heartbeat %s" % (self.address))
                        except:
                            rospy.loginfo("could not connect to hertbeat %s" % (self.address))
                            return

                        self.ready = True

                        self.last_heartbeat = time.time()
                        timeout = 5.0
                        while not rospy.is_shutdown() and time.time() - self.last_heartbeat < timeout and self.alive:
                            if time.time() - self.last_heartbeat > timeout/2.0:
                                rospy.loginfo(
                                    "Reconnecting in %.1f seconds %s" % (timeout - (time.time() - self.last_heartbeat), self.address))
                                #self.check_find_person_service()
                            time.sleep(0.5)
                    except pygatt.exceptions.BLEError:
                        rospy.loginfo("device disconnected")
        except:
            rospy.logerr(traceback.format_exc())
        finally:
            self.stop()
            self.mgr.on_terminate(self)

    def check_find_person_service(self):
        if rospy.Time.now() - self.last_check_find_person > rospy.Duration(5):
            self.last_check_find_person = rospy.Time.now()
            try:
                rospy.wait_for_service("/faceapp/find_person", timeout=5)
                rospy.loginfo("find person service found")
                if self.find_person_proxy is None:
                    self.find_person_proxy = rospy.ServiceProxy("/faceapp/find_person", faceapp_msgs.srv.FindPerson)
                self.target.char_write(self.find_person_ready_uuid, value=array.array('B', "True"))
            except:
                rospy.loginfo("could not find faceapp find_person service, retry in 5 seconds")
                self.find_person_proxy = None
                self.target.char_write(self.find_person_ready_uuid, value=array.array('B', "False"))

    def buffered_callback(self, handle, value):
        buf = self.data_buffer[handle]

        if value[-1] == ord('\n'):
            result = json.loads(str(buf + value[:-1]))
            # TODO
        else:
            self.data_buffer[handle] = buf + value

    def destination_callback(self, handle, value):
        rospy.loginfo("destination_callback {}".format(value))
        msg = std_msgs.msg.String()
        msg.data = str(value)

        if msg.data == "__cancel__":
            rospy.loginfo("cancel navigation")
            event = NavigationEvent(subtype="cancel", param=None)
            self.eventPub.publish(str(event))
            return

        rospy.loginfo("destination: " + msg.data)
        event = NavigationEvent(subtype="destination", param=msg.data)
        self.eventPub.publish(str(event))

    def summons_callback(self, handle, value):
        rospy.loginfo("summons_callback {}".format(value))
        msg = std_msgs.msg.String()
        msg.data = str(value)

        rospy.loginfo("summons: " + msg.data)
        event = NavigationEvent(subtype="summons", param=msg.data)
        self.eventPub.publish(str(event))

    def find_person_callback(self, handle, value):
        rospy.loginfo("find_person_callback {} <{}>".format(value, type(value)))
        value=str(value)
        try:
            (name, timeout) = value.split(";")
            timeout = int(timeout)
        except:
            self.target.char_write(self.find_person_callback_uuid, value="could not parse input: "+value)
            return

        self._find_person(name, timeout)

    @util.setInterval(0.1, times=1)
    def _find_person(self, name, timeout):
        rospy.loginfo("find_person: %s, timeout=%dms", name, timeout)
        try:
            res = self.find_person_proxy(name, timeout)
            rospy.loginfo("find_person: %s", str(res))
        except:
            rospy.logerr(traceback.format_exc())

    def heartbeat_callback(self, handle, value):
        #rospy.loginfo(("heartbeat(%s):" % self.address) +  str(value))
        self.last_heartbeat = time.time()

    def req_stop(self):
        self.alive = False

    def stop(self):
        self.alive = False
        self.ready = False
        if self.target is not None:
            try:
                self.target.disconnect()
            except pygatt.exceptions.BLEError:
                #device is already closed #rospy.loginfo("device disconnected")
                pass

        self.adapter.stop()

    def handleSpeak(self, req):
        if not self.ready:
            return None

        if req.force:
            req.text = "__force_stop__\n" + req.text

        self.call_async(self.speak_uuid, req.text)

        return cabot_msgs.srv.SpeakResponse(True)


    def _event_callback(self, msg):
        event = BaseEvent.parse(msg.data)
        if event is None:
            rospy.logerr("cabot event %s cannot be parsed", msg.data)
            return

        if event.type != NavigationEvent.TYPE:
            return

        if event.subtype == "next":
            # notify the phone next event
            self.call_async(self.navi_uuid, "next")

        if event.subtype == "arrived":
            self.call_async(self.navi_uuid, "arrived")

        if event.subtype == "content":
            self.call_async(self.content_uuid, event.param)

        if event.subtype == "sound":
            self.call_async(self.sound_uuid, event.param)


    @util.setInterval(0.01, times=1)
    def call_async(self, uuid, text):
        rospy.loginfo("call async %s with %s", uuid, text)
        try:
            self.target.char_write(uuid, value=array.array('B', text))
        except:
            try:
                self.target.char_write(uuid, value=array.array('B', text))
            except:
                return


class AnyDevice(gatt.Device,object):
    """
    An implementation of ``gatt.Device`` that connects to any GATT device
    and prints all services and characteristics.
    """

    def __init__(self, mac_address, manager, auto_reconnect=False):
        super(AnyDevice,self).__init__(mac_address=mac_address, manager=manager)
        self.auto_reconnect = auto_reconnect

    def connect(self):
        print("Connecting...")
        super(AnyDevice,self).connect()

    def connect_succeeded(self):
        super(AnyDevice,self).connect_succeeded()
        print("[%s] Connected" % (self.mac_address))

    def connect_failed(self, error):
        super(AnyDevice,self).connect_failed(error)
        print("[%s] Connection failed: %s" % (self.mac_address, str(error)))

    def disconnect_succeeded(self):
        super(AnyDevice,self).disconnect_succeeded()

        print("[%s] Disconnected" % (self.mac_address))
        if self.auto_reconnect:
            self.connect()

    def services_resolved(self):
        super(AnyDevice,self).services_resolved()

        print("[%s] Resolved services" % (self.mac_address))
        for service in self.services:
            print("[%s]  Service [%s]" % (self.mac_address, service.uuid))
            for characteristic in service.characteristics:
                print("[%s]    Characteristic [%s]" % (self.mac_address, characteristic.uuid))

class AnyDeviceManager(gatt.DeviceManager, object):
    def __init__(self, adapter_name, team=None):
        super(AnyDeviceManager, self).__init__(adapter_name = adapter_name)
        self.team = "CaBot" + ("-" + team if team is not None else "")
        print("team: " + self.team)
        self.bles = {}
        rospy.Service("/speak", cabot_msgs.srv.Speak, self.handleSpeak)

    def handleSpeak(self, req):
        ret = cabot_msgs.srv.SpeakResponse(False)
        for ble in self.bles.values():
            temp = ble.handleSpeak(req=req)
            if temp:
                ret = temp
        return ret

    def on_terminate(self, bledev):
        print("terminate {}".format(bledev.address))
        self.bles.pop(bledev.address)

    def make_device(self, mac_address):
        return AnyDevice(mac_address=mac_address, manager=self)

    def device_discovered(self, device):
        if device.alias() == self.team:#"CaBot":
            if not device.mac_address in self.bles.keys():
                for service in device.services:
                    print("[%s]  Service [%s]" % (device.mac_address, service.uuid))
                ble = CaBotBLE(address=device.mac_address, mgr=self)
                self.bles[device.mac_address] = ble
                thread = threading.Thread(target=ble.start)
                thread.start()

    def stop(self):
        for ble in self.bles.values():
            ble.req_stop()


if __name__ == "__main__":
    rospy.init_node("cabot_ble_node")
    
    team = rospy.get_param("~team", None)
    adapter_name = rospy.get_param("~adapter", "hci0")

    manager = AnyDeviceManager(adapter_name=adapter_name, team=team)

    # power on the adapter
    if not manager.is_adapter_powered:
        manager.is_adapter_powered = True

    manager.start_discovery(service_uuids=["35CE0000-5E89-4C0D-A3F6-8A6A507C1BF1"])

    try:
        manager.run()
    except:
        rospy.loginfo(traceback.format_exc())
    finally:
        manager.stop()
        manager._main_loop.quit()

    #rospy.spin()
