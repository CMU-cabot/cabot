#!/usr/bin/env python

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
# -*- coding: utf-8 -*-

import json
import argparse

import rospy
import tf2_ros
import tf_conversions
from std_msgs.msg import String

class BeaconMapper:
    def __init__(self):
        self._current_position = None
        self._fingerprints = []
        self._count = 0

    def beacons_callback(self,message):
        beacons_obj = json.loads(message.data)
        #print(beacons_obj)
        if self._current_position is not None:
            #print(self._current_position)
            t = self._current_position
            fp_data = {
                "information": {
                    "x": t.transform.translation.x,
                    "y": t.transform.translation.y,
                    "z": t.transform.translation.z,
                    "tags":[
                        beacons_obj["phoneID"]
                    ],
                    "rotation":{
                        "x": t.transform.rotation.x,
                        "y": t.transform.rotation.y,
                        "z": t.transform.rotation.z,
                        "w": t.transform.rotation.w
                    }
                },
                "data":{
                    "timestamp": beacons_obj["timestamp"],
                    "beacons": beacons_obj["data"]
                }
            }
            self._fingerprints.append(fp_data)
            self._count += 1
            print("sampling data count = " + str(self._count) )

    def set_current_position(self, position):
        self._current_position = position

if __name__ == "__main__":
    rospy.init_node('tf2_beacons_listener')

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    sub_topics_str = rospy.get_param("~topics", "['/wireless/beacons','/wireless/wifi']")
    import ast
    sub_topics = ast.literal_eval(sub_topics_str)

    mapper = BeaconMapper()
    #beacons_sub = rospy.Subscriber("/wireless/beacons", String, mapper.beacons_callback)
    #wifi_sub = rospy.Subscriber("/wireless/wifi", String, mapper.beacons_callback)
    subscribers = []
    for sub_topic in sub_topics:
        print("set " + sub_topic +  " subscriber." )
        sub = rospy.Subscriber(sub_topic, String, mapper.beacons_callback)
        subscribers.append(sub)

    r = rospy.Rate(100) # 100 Hz

    output = rospy.get_param("~output")

    def shutdown_hook():
        if output is not None and 0<len(mapper._fingerprints):
            with open(output, "w") as f:
                json.dump( mapper._fingerprints, f)

    rospy.on_shutdown(shutdown_hook)

    while not rospy.is_shutdown():
        rostime = rospy.get_time()
        try:
            t = tfBuffer.lookup_transform('map', 'base_link', rospy.Time(0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logerr('LookupTransform Error')
            rospy.sleep(1.0)
            continue

        #translation = (t.transform.translation.x,
        #                t.transform.translation.y,
        #                t.transform.translation.z)

        #print(rostime ,translation)

        mapper.set_current_position(t)
        r.sleep()
