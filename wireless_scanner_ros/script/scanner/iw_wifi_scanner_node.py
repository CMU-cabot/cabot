#!/usr/bin/python
# -*- coding: utf-8 -*-

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

import subprocess

import rospy
from std_msgs.msg import String

def run_command(cmd):
    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    out = proc.stdout.read().decode('utf-8')
    err = proc.stderr.read().decode('utf-8')
    return out, err

if __name__ == "__main__":
    # This node must be run with sudo permission
    # examle
    #    $ sudo -s
    #    $ rosrun wireless_scanner_ros iw_wifi_scanner_node.py

    rospy.init_node("iw_wifi_scanner")
    interface = rospy.get_param("~interface", "wlan0")
    pub = rospy.Publisher("wireless/wifi_iw_scan_str", String, queue_size = 10)

    command = ["iw", interface, "scan"]
    while not rospy.is_shutdown():
        out,err = run_command(command) # scan
        if len(err) == 0: # no error
            string = String()
            string.data = out
            pub.publish(string)
        else: # error
            cmd_str = " ".join(command)
            rospy.logerr( "["+ cmd_str + "] " + err)
            rospy.sleep(0.1)
