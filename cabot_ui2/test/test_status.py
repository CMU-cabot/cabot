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
"""Test cabot_ui.event module"""

import unittest
import cabot_ui.status


class TestStatus(unittest.TestCase):
    """Test class"""

    def setUp(self):
        pass

    def test_basic(self):
        with self.assertRaises(NotImplementedError) as context:
            cabot_ui.status.StatusManager()
        self.assertIsNotNone(context)

    def test_singleton(self):
        i1 = cabot_ui.status.StatusManager.get_instance()
        i2 = cabot_ui.status.StatusManager.get_instance()

        self.assertEqual(i1, i2)
        self.assertEqual(i1.state, cabot_ui.status.State.idle)
