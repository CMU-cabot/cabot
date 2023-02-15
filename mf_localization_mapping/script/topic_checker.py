#!/usr/bin/python3
# -*- coding: utf-8 -*-

# Copyright (c) 2021, 2023  IBM Corporation and Carnegie Mellon University
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
import io
from timeout_thread import TimeoutThread
import argparse
import time


class colors:
    BLUE = '\033[36m'
    RED = '\033[31m'
    ENDC = '\033[0m'


class TopicChecker:
    def __init__(self, timeout=3):
        self.timeout = timeout
        self.topics = []
        self.identifiers = []
        self.topic_types = []
        self.subps = []
        self.threads = []

    def add_topic(self, topic, identifier, topic_type):
        self.topics.append(topic)
        self.identifiers.append(identifier)
        self.topic_types.append(topic_type)

    def run_subprocesses(self):
        for topic, topic_type in zip(self.topics, self.topic_types):
            sub_p = subprocess.Popen(
                ['ros2', 'topic', 'echo', topic, topic_type],
                stdout=subprocess.PIPE,
                stderr=subprocess.DEVNULL
            )
            self.subps.append(sub_p)

        for subp in self.subps:
            th_cl = TimeoutThread(subp, self.timeout)
            th_cl.daemon = True
            th_cl.start()
            self.threads.append(th_cl)

    def check_topics(self):
        result = True
        for i in range(len(self.topics)):
            topic = self.topics[i]
            iden = self.identifiers[i]
            subp = self.subps[i]

            found = False
            for line in io.TextIOWrapper(subp.stdout, encoding="utf-8"):
                if line.find(iden) != -1:
                    found = True
            result = result*found
            print(F"{colors.BLUE if found else colors.RED}{topic} topic was {(not found)*'NOT '}found{colors.ENDC}")
        return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--timeout", type=int, default=3)
    args = parser.parse_args()

    timeout = args.timeout

    while True:
        print("Checking if required topics are published or not.")
        checker = TopicChecker(timeout)
        checker.add_topic("/velodyne_points", "is_dense", "sensor_msgs/msg/PointCloud2")
        checker.add_topic("/imu/data", "linear_acceleration", "sensor_msgs/msg/Imu")
        checker.add_topic("/esp32/wifi_scan_str", ",", "std_msgs/msg/String")
#        checker.add_topic("/wireless/wifi", "rssi")
        checker.add_topic("/wireless/beacon_scan_str", "rssi", "std_msgs/msg/String")
        checker.add_topic("/wireless/beacons", "rssi", "std_msgs/msg/String")
        checker.run_subprocesses()
        result = checker.check_topics()
        print(F"{colors.BLUE if result else colors.RED}Setup {(not result)*'NOT '}completed{colors.ENDC}")
        print("")
        time.sleep(timeout)


if __name__ == "__main__":
    main()
