#!/usr/bin/python

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

import subprocess
from timeout_thread import *
import argparse
import time

class TopicChecker:
    def __init__(self, timeout=3):
        self.timeout = timeout
        self.topics = []
        self.identifiers = []
        self.subps = []
        self.threads = []

    def add_topic(self, topic, identifier):
        self.topics.append(topic)
        self.identifiers.append(identifier)

    def run_subprocesses(self):
        for topic in self.topics:
            sub_p = subprocess.Popen(
                                 ['rostopic', 'echo', topic],
                                 stdout = subprocess.PIPE
                                 )
            self.subps.append(sub_p)

        for subp in self.subps:
            th_cl = TimeoutThread(subp, self.timeout)
            th_cl.setDaemon(True)
            th_cl.start()
            self.threads.append(th_cl)

    def check_topics(self):
        result = True
        for i in xrange(len(self.topics)):
            topic = self.topics[i]
            iden = self.identifiers[i]
            subp = self.subps[i]

            found = False
            for line in iter(subp.stdout.readline,''):
                if line.find(iden) != -1:
                    found = True
            result = result*found
            print topic + " topic was " + (not found)*"NOT " +  "found"
        return result

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--timeout", type=int, default=3)
    args = parser.parse_args()

    timeout = args.timeout

    while True:
        print "Checking if required topics are published or not."
        checker = TopicChecker(timeout)
        checker.add_topic("/velodyne_points", "is_dense")
        checker.add_topic("/imu/data", "linear_acceleration")
        checker.add_topic("/wireless/wifi_iw_scan_str", "BSS")
        checker.add_topic("/wireless/wifi", "rssi")
        checker.add_topic("/wireless/beacon_scan_str", "rssi")
        checker.add_topic("/wireless/beacons", "rssi")
        checker.run_subprocesses()
        result = checker.check_topics()
        print "Setup" + (not result)*" NOT"  +  " completed"
        print ""
        time.sleep(timeout)

if __name__=="__main__":
    main()
    print "Press Enter key to quit."
    input = raw_input()
