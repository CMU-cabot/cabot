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
"""Test turn detector module"""

import os
import sys
import unittest
import roslib
import json
import rospy
import people_msgs.msg
import sensor_msgs.msg
import geometry_msgs.msg
import rosbag
import tf2_msgs.msg
import threading

PKG = 'cabot_navigation'
roslib.load_manifest(PKG)
sys.path.append('../src')

from cabot_ui import geojson, datautil
from cabot_ui.navigation import Navigation

sleep_time = 0.1
## test

class TestPeopleObstacleLayer(unittest.TestCase):
    """Test class"""

    def setUp(self):
        print "setUp"
        self.dir_path = os.path.dirname(os.path.realpath(__file__))

    def test_from_bag(self):
        rospy.loginfo("test_from_bag")
        
        pub_scan = rospy.Publisher("/scan", sensor_msgs.msg.LaserScan, queue_size=10)
        pub_people = rospy.Publisher("/people", people_msgs.msg.People, queue_size=10)

        for key in range(2, 12, 2):
            self.bag = rosbag.Bag(os.path.join(self.dir_path, 'data', 'people_obstacle', "{}m.bag".format(key)))
            init_tf()
            for topic, msg, t in self.bag.read_messages(topics=['/scan', '/people', '/tf']):
                if rospy.is_shutdown():
                    break
            
                if topic == '/scan':
                    #rospy.loginfo("pub "+topic)
                    msg.header.stamp = rospy.get_rostime()
                    pub_scan.publish(msg)
                if topic == '/people':
                    #rospy.loginfo("pub people "+msg.header.frame_id)
                    msg.header.stamp = rospy.get_rostime()
                    pub_people.publish(msg)
                    rospy.sleep(sleep_time)
                if topic == "/tf":
                    global tf_buffer
                    process_tf(msg, tf_buffer)


tf_buffer = {}
def init_tf():
    global tf_buffer
    tf_buffer = {}
    msg = geometry_msgs.msg.TransformStamped()
    msg.header.frame_id = "base_footprint"
    msg.child_frame_id = "velodyne"
    msg.transform.translation.x = 0
    msg.transform.translation.y = 0
    msg.transform.translation.z = 0
    msg.transform.rotation.x = 0
    msg.transform.rotation.y = 0
    msg.transform.rotation.z = 0
    msg.transform.rotation.w = 1
    tf_buffer["base_footprint-velodyne"] = msg
    
    msg = geometry_msgs.msg.TransformStamped()
    msg.header.frame_id = "base_footprint"
    msg.child_frame_id = "lidar_link"
    msg.transform.translation.x = 0
    msg.transform.translation.y = 0
    msg.transform.translation.z = 0
    msg.transform.rotation.x = 0
    msg.transform.rotation.y = 0
    msg.transform.rotation.z = 0
    msg.transform.rotation.w = 1
    tf_buffer["base_footprint-lidar_link"] = msg

lock = threading.Lock()
def process_tf(msg, buffer):
    lock.acquire()
    for t in msg.transforms:
        key = "{}-{}".format(t.header.frame_id, t.child_frame_id)
        buffer[key] = t
    lock.release()

def tf_publisher():
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        lock.acquire()

        msg = tf2_msgs.msg.TFMessage()
        for key in tf_buffer:
            tf = tf_buffer[key]
            tf.header.stamp = rospy.get_rostime()
            if tf.transform.translation.z == 45:
                tf.transform.translation.z = 0
            msg.transforms.append(tf)
        pubtf.publish(msg)

        lock.release()
        rate.sleep()

if __name__ == "__main__":
    import rosunit
    rospy.init_node("test_people_obstacle_layer")

    pubtf = rospy.Publisher("/tf", tf2_msgs.msg.TFMessage, queue_size=100, latch=True)
    thread = threading.Thread(target=tf_publisher)
    thread.start()

    sleep_time = rospy.get_param("~sleep_time", 0.1)
    rospy.loginfo("sleep_time {}".format(sleep_time))
    rosunit.unitrun(PKG, 'test_people_obstacle_layer', TestPeopleObstacleLayer)
    while True:
        rospy.sleep(1)
