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
# -*- coding: utf-8 -*-

import sys
import unittest
import traceback

PKG='cabot_ui'
import rospy
import roslib
import cabot_msgs.srv
roslib.load_manifest(PKG)
sys.path.append('../src')
import cabot_ui.tts as tts


class TestTTS(unittest.TestCase):
    def test_srv(self):
        try:
            rospy.wait_for_service('/speak', timeout=5)
        except rospy.ROSException:
            self.fail("service timeout")

    def test_speak(self):
        try:
            rospy.wait_for_service('/speak', timeout=5)
            tts.speak("Hello world", force=False)
            rospy.sleep(3)
        except rospy.ROSException:
            self.fail("service timeout")

    def test_speak_rate(self):
        try:
            rospy.wait_for_service('/speak', timeout=5)
            tts.speak("Hello world", rate=100, force=False)
            rospy.sleep(3)            
        except rospy.ROSException:
            self.fail("service timeout")

    def test_japanese(self):
        try:
            rospy.wait_for_service('/speak', timeout=5)
            tts.speak(u"ハローワールド", force=False, lang="ja")
            rospy.sleep(3)
        except rospy.ROSException:
            self.fail("service timeout")

if __name__ == "__main__":
    import rosunit
    rosunit.unitrun(PKG, 'test_tts', TestTTS)
