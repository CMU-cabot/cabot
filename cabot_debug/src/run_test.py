#!/usr/bin/env python3

###############################################################################
# Copyright (c) 2019, 2022  Carnegie Mellon University
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
###############################################################################

import os
import sys
import re
import numpy
import time
import traceback
import uuid
import multiprocessing
from pathlib import Path
import yaml
import logging

from optparse import OptionParser
from matplotlib import pyplot as plt
import rclpy
import rclpy.node
from rosidl_runtime_py import set_message_fields

from cabot_common.rosbag2 import BagReader

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s.%(msecs)03d [%(levelname)s]: %(message)s',
    datefmt='%Y-%m-%d %H:%M:%S',
)

def import_class(input_str):
    import importlib
    # Split the input string and form module and class strings
    module_str, class_str = input_str.rsplit('/', 1)
    module_str = module_str.replace('/', '.')
    # Import the module dynamically
    module = importlib.import_module(module_str)
    return getattr(module, class_str)


class Tester:
    def __init__(self, node):
        self.node = node
        self.done = False
        self.alive = True
        self.subscriptions = {}
        self.timers = {}

    def handle_case(self, test_case):
        logging.info(f"Test: {test_case['name']}")
        self.done = False
        test_action = test_case['action']
        test_action['uuid'] = uuid.uuid4()
        action_type = test_action['type']

        test_action_method = getattr(self, action_type, None)
        if callable(test_action_method):
            return test_action_method(test_action)
        else:
            logging.error(f"unknown test type {action_type}")

    def check_topic_error(self, test_action):
        logging.info(test_action)
        topic = test_action['topic']
        topic_type = test_action['topic_type']
        topic_type = import_class(topic_type)
        condition = test_action['condition']
        uuid = test_action['uuid']

        def topic_callback(msg):
            try:
                context = {'msg': msg}
                exec(f"result=({condition})", context)
                if context['result']:
                    logging.error(f"check_topic_error: condition ({condition}) matched\n{msg}")
                    self.alive = False
                    sub = self.subscriptions[uuid]
                    self.node.destroy_subscription(sub)
            except:
                logging.error(traceback.format_exc())

        sub = self.node.create_subscription(topic_type, topic, topic_callback, 10)
        self.subscriptions[uuid] = sub

    def wait_topic(self, test_action):
        logging.info(test_action)
        topic = test_action['topic']
        topic_type = test_action['topic_type']
        topic_type = import_class(topic_type)
        condition = test_action['condition']
        timeout = test_action['timeout']
        uuid = test_action['uuid']

        def topic_callback(msg):
            try:
                context = {'msg': msg}
                exec(f"result=({condition})", context)
                if context['result']:
                    self.done = True
                    sub = self.subscriptions[uuid]
                    self.node.destroy_subscription(sub)
            except:
                logging.error(traceback.format_exc())
        sub = self.node.create_subscription(topic_type, topic, topic_callback, 10)
        self.subscriptions[uuid] = sub
        return timeout

    def pub_topic(self, test_action):
        logging.info(test_action)
        topic = test_action['topic']
        topic_type = test_action['topic_type']
        topic_type = import_class(topic_type)
        message = test_action['message']

        msg = topic_type()
        data = yaml.safe_load(message)
        set_message_fields(msg, data)

        pub = self.node.create_publisher(topic_type, topic, 10)
        pub.publish(msg)
        self.node.destroy_publisher(pub)
        self.done = True
        return 0

    def wait(self, test_action):
        logging.info(test_action)
        seconds = test_action['seconds']
        uuid = test_action['uuid']

        def timer_callback():
            self.done = True
            timer = self.timers[uuid]
            timer.cancel()
            self.node.destroy_timer(timer)

        timer = self.node.create_timer(seconds, timer_callback)
        self.timers[uuid] = timer
        return seconds*2

    def terminate(self, test_action):
        logging.info(test_action)
        sys.exit(0)


def main():
    parser = OptionParser(usage="""
    Example
    {0} -f <test yaml>                     # run test
    """.format(sys.argv[0]))

    parser.add_option('-f', '--file', type=str, help='test yaml file')

    (options, args) = parser.parse_args()

    if not options.file:
        parser.print_help()
        sys.exit(0)

    rclpy.init()
    node = rclpy.node.Node("test_node")
    tester = Tester(node)

    with open(options.file) as file:
        test_cases = yaml.safe_load(file)

        for case in test_cases['checks']:
            timeout = tester.handle_case(case)

        for case in test_cases['tests']:
            if not tester.alive:
                logging.error("Tester is terminated")
                break
            timeout = tester.handle_case(case)
            timeout = timeout if timeout is not None else 60
            start = time.time()
            logging.info(f"Timeout = {timeout} seconds, done={tester.done}, alive={tester.alive}")
            while tester.alive and not tester.done and time.time() - start < timeout:
                rclpy.spin_once(node, timeout_sec=1)
            if not tester.done:
                logging.error("Timeout")
                sys.exit(1)

if __name__ == "__main__":
    main()
