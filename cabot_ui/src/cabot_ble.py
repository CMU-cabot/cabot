#!/usr/bin/env python

# Copyright (c) 2020 Carnegie Mellon University
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

import binascii
from collections import deque
import array
import pygatt
import rospy
import time
import sys
import json
import std_msgs.msg
import cabot_msgs.srv
from cabot_ui.event import NavigationEvent

from pygatt.util import uuid16_to_uuid
from uuid import UUID
from pygatt.backends import BLEBackend, Characteristic, BLEAddressType

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
    ADDRESS = "CC:08:8D:25:19:7E"
    
    def __init__(self):
        self.dest_uuid = UUID(CaBotBLE.UUID_FORMAT.format(0x10))
        self.speak_uuid = UUID(CaBotBLE.UUID_FORMAT.format(0x200))
        self.heartbeat_uuid = UUID(CaBotBLE.UUID_FORMAT.format(0x9999))
        self.data_buffer = {}
        self.eventPub = rospy.Publisher('/cabot/event', std_msgs.msg.String, queue_size=1)

        self.adapter = pygatt.GATTToolBackend()
        self.target = None
        self.last_heartbeat = time.time()
        
        # speak
        rospy.Service("/speak", cabot_msgs.srv.Speak, self.handleSpeak)
        self.queue = deque([])

    def start(self):
        try:
            self.adapter.start(reset_on_start=False)
            target = None

            address = CaBotBLE.ADDRESS
            try:
                rospy.loginfo("trying to connect to %s" % (address))
                target = self.adapter.connect(address, timeout=15)
                target.exchange_mtu(64)
            except pygatt.exceptions.NotConnectedError:
                rospy.loginfo("device not found")

            if target is not None:
                try:
                    self.target = target
                    self.target.subscribe(self.dest_uuid, self.destination_callback, indication=False)
                    rospy.loginfo("subscribed to destination")
                    self.target.subscribe(self.heartbeat_uuid, self.heartbeat_callback, indication=False)
                    rospy.loginfo("subscribed to heartbeat")
                except pygatt.exceptions.BLEError:
                    rospy.loginfo("device disconnected")
                
        except:
            import traceback
            traceback.print_exc()

        self.last_heartbeat = time.time()
        while not rospy.is_shutdown() and time.time() - self.last_heartbeat < 5:
            if time.time() - self.last_heartbeat > 3:
                rospy.loginfo("Reconnecting in %d seconds " %(int(round(5 - (time.time() - self.last_heartbeat)))))
            time.sleep(1)

        if not rospy.is_shutdown():
            rospy.loginfo("trying to reconnect")
            self.start()

    def buffered_callback(self, handle, value):
        buf = self.data_buffer[handle]

        if value[-1] == ord('\n'):
            result = json.loads(str(buf+value[:-1]))
            #TODO
        else:
            self.data_buffer[handle] = buf+value

    def destination_callback(self, handle, value):
        msg = std_msgs.msg.String()
        msg.data = str(value)

        if msg.data == "__cancel__":
            rospy.loginfo("cancel navigation")
            event = NavigationEvent(subtype="cancel", param=None)
            self.eventPub.publish(str(event))
            return

        rospy.loginfo("destination: "+msg.data)
        event = NavigationEvent(subtype="destination", param=msg.data)
        self.eventPub.publish(str(event))

    def heartbeat_callback(self, handle, value):
        rospy.loginfo("heartbeat: "+str(value))
        self.last_heartbeat = time.time()

    def stop(self):
        if self.target is not None:
            self.target.disconnect()
        self.adapter.stop()

    def handleSpeak(self, req):
        if req.force:
            req.text = "__force_stop__\n"+req.text
        
        data = array.array('B', req.text)
        try:
            self.target.char_write(self.speak_uuid, value=data)
        except:
            self.start()
            self.target.char_write(self.speak_uuid, value=data)

        return cabot_msgs.srv.SpeakResponse(True)


if __name__ == "__main__":
    rospy.init_node("cabot_ble_node")
    ble = CaBotBLE()
    ble.start()
    ble.stop()
    

