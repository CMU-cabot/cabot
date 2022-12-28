#!/usr/bin/env python3
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

import numpy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSDurabilityPolicy
from std_msgs.msg import Float64


class AltitudeManager():
    def __init__(self, node, verbose=False):
        self.node = node
        self.logger = node.get_logger()
        self.queue = []
        self.verbose = verbose
        self.queue_limit = 60
        self.timestamp_interval_limit = 3.0
        self.window_size = 20
        self.threshold = 0.3 * 12

        self.initial_pressure = None
        self.pressure_std_pub = self.node.create_publisher(
            Float64, "pressure_std", QoSProfile(depth=10, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL))

    def put_pressure(self, pressure):
        if not self.initial_pressure:
            self.initial_pressure = pressure

        if self.queue_limit <= len(self.queue):
            self.queue.pop(0)

        if len(self.queue) == 0:
            self.queue.append(pressure)
            return

        if (self.queue[-1].header.stamp - pressure.header.stamp).to_sec() > self.timestamp_interval_limit:
            if self.verbose:
                self.logger.error(F"timestamp interval between two altimters is too large ({self.timestamp_interval_limit} sec). "
                                  "AltitudeManager was reset.")
            self.queue.clear()

        self.queue.append(pressure)

    def is_height_changed(self):
        if len(self.queue) < self.window_size:
            return False

        relative = []
        for i in range(1, self.window_size+1):
            relative.append(
                self.queue[-i].fluid_pressure - self.initial_pressure.fluid_pressure)

        stdev = numpy.std(relative)

        msg = Float64()
        msg.data = stdev
        self.pressure_std_pub.publish(msg)

        if self.verbose:
            self.logger.info("Altimeter changed: {}".format(stdev))

        if self.threshold < stdev:
            return True

        return False
