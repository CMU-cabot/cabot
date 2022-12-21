#!/usr/bin/env python3

# Copyright (c) 2020, 2022  Carnegie Mellon University
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
"""Test geoutil module"""

import unittest
import math
import time

from cabot_ui import geoutil
from tf_transformations import quaternion_from_euler


class TestGeoutil(unittest.TestCase):
    """Test class"""

    def setUp(self):
        pass

    def test_conversion(self):
        """test local and global conversion"""
        a = geoutil.Anchor(lat=40, lng=-80, rotate=0)
        g = geoutil.Latlng(lat=40.001, lng=-80.001)

        local = geoutil.global2local(g, a)
        g2 = geoutil.local2global(local, a)

        print(g)
        print(g2)
        self.assertAlmostEqual(g.lat, g2.lat)
        self.assertAlmostEqual(g.lng, g2.lng)

    def test_quaternion(self):
        q1 = quaternion_from_euler(0.0, 0.0, 0.0)
        q2 = quaternion_from_euler(0.0, 0.0, math.pi)

        pos1 = [0.0, 0.0, 0.0]
        pos2 = [10.0, 10.0, 0.0]

        p1 = geoutil.msg_from_pq(pos1, q1)
        p2 = geoutil.msg_from_pq(pos2, q2)

        self.assertFalse(geoutil.in_angle(p1, p2, 10))
        self.assertTrue(geoutil.in_angle(p1, p2, 46))

    def test_conversion_time(self):
        """test local and global conversion"""
        a = geoutil.Anchor(lat=40, lng=-80, rotate=0)
        g = geoutil.Latlng(lat=40.001, lng=-80.001)

        s = time.time()
        for i in range(0, 1000):
            local = geoutil.global2local(g, a)
            _ = geoutil.local2global(local, a)
        self.assertLess(time.time()-s, 1)

    def test_diff_angle(self):
        p0 = geoutil.Pose(x=0, y=0, r=0)
        for r in range(0, 101):
            rad = r/100.0*2*math.pi-math.pi
            p1 = geoutil.Pose(x=0, y=0, r=rad)
            diff = geoutil.get_rotation(p1.orientation, p0.orientation)

            print("current=%.2f, target=%.2f, diff=%.2f" % (rad, p0.r, diff))


