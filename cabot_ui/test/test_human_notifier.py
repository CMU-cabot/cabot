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
"""Test navcognode module"""

import sys
import unittest
import traceback

import rospy
import roslib
import std_msgs.msg
import tf
import threading
import json

from cabot_ui import geojson, datautil, geoutil

from cabot_ui import menu

PKG = 'cabot_ui'
roslib.load_manifest(PKG)
sys.path.append('../src')

## test

class TestHumanNotifier(unittest.TestCase):
    """Test class"""
    
    def test_pause_navigation(self):
        rospy.sleep(1100)
        
def test():
    rosunit.unitrun(PKG, 'test_human_notifier', TestHumanNotifier)
    rospy.signal_shutdown("finish")
        
if __name__ == "__main__":
    rospy.init_node("test_human_notifier")
    import rosunit

    t = threading.Thread(target=test)
    t.start()

