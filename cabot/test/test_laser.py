#!/usr/bin/python

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

"""Test laser"""

import os
import re
import sys
import threading
import mutex

import unittest
import traceback

import rospy
import roslib
import rosbag
import sensor_msgs.msg
import tf2_msgs.msg

PKG = 'cabot'
roslib.load_manifest(PKG)
sys.path.append('../src')

## test

class TestLaser(unittest.TestCase):
    """Test class"""

    def setUp(self):
        pass

    def load_bags(self, subdir):
        dir = os.path.join(os.path.dirname(os.path.realpath(__file__)), "data", subdir)
        bags = []
        for f in [f for f in os.listdir(dir) if re.match(".*bag", f)]:
            bags.append(self.load_bag(dir, f))
        return bags

    def load_bag(self, dir, name):

        return rosbag.Bag(os.path.join(dir, name))

    def test_scan(self):
        bags = self.load_bags("set4")
        for bag in bags:
            rospy.loginfo("testing {}".format(bag))
            count = 0
            for topic, msg, t in bag.read_messages(topics=['/scan', '/tf', '/tf_static']):
                if rospy.is_shutdown():
                    return
                
                if topic == "/scan":
                    if count > 0:
                        count -=1
                        continue
                    msg.header.stamp = rospy.get_rostime()
                    pub.publish(msg)
                    rospy.sleep(duration)
                if topic == "/tf":
                    global tf_buffer
                    process_tf(msg, tf_buffer)
                if topic == "/tf_static":
                    global tfs_buffer
                    process_tf(msg, tfs_buffer)
                    pass
                
                
        self.assertTrue(True)

        
tf_buffer = {}
tfs_buffer = {}

def process_tf(msg, buffer):
    lock = threading.Lock()
    lock.acquire()
    for t in msg.transforms:
        key = "{}-{}".format(t.header.frame_id, t.child_frame_id)
        buffer[key] = t
    lock.release()

def tf_publisher():
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        lock = threading.Lock()
        lock.acquire()

        if len(tfs_buffer) > 0:
            msg_static = tf2_msgs.msg.TFMessage()
            for key in tfs_buffer:
                tfs = tfs_buffer[key]
                tfs.header.stamp = rospy.get_rostime()
                msg_static.transforms.append(tfs)
                
            pubtfs.publish(msg_static)
            tfs_buffer.clear()

        msg = tf2_msgs.msg.TFMessage()
        for key in tf_buffer:
            tf = tf_buffer[key]
            tf.header.stamp = rospy.get_rostime()
            msg.transforms.append(tf)
        pubtf.publish(msg)

        lock.release()
        rate.sleep()

if __name__ == "__main__":
    import rosunit
    rospy.init_node("test_laser")
    pub = rospy.Publisher("/scan", sensor_msgs.msg.LaserScan, queue_size=10, latch=True)
    pubtf = rospy.Publisher("/tf", tf2_msgs.msg.TFMessage, queue_size=100, latch=True)
    pubtfs = rospy.Publisher("/tf_static", tf2_msgs.msg.TFMessage, queue_size=100, latch=True)
    thread = threading.Thread(target=tf_publisher)
    thread.start()
    duration = rospy.get_param("~duration", 3)
    rosunit.unitrun(PKG, 'test_laser', TestLaser)
