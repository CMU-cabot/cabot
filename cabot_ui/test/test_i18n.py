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
# -*- coding: utf-8 -*-
"""Test cabot_ui.i18n module"""

import os
import rclpy
import rclpy.node
import unittest
import cabot_ui.i18n
from cabot_ui.cabot_rclpy_util import CaBotRclpyUtil


class TestI18N(unittest.TestCase):
    """Test class"""
    @classmethod
    def setUpClass(cls):
        # Initialize the ROS context for the test node
        rclpy.init()
        node = rclpy.node.Node('test_i18n')
        CaBotRclpyUtil.initialize(node)

    def setUp(self):
        cabot_ui.i18n.init(os.path.join("test", "i18n"))

    def test_basic(self):
        """test i18n"""

        self.assertEqual(cabot_ui.i18n.localized_string("test", lang="ja"), u'テスト')
        self.assertEqual(cabot_ui.i18n.localized_string("test", "en"), "Test")
        self.assertEqual(cabot_ui.i18n.localized_string("test"), "Test")
        self.assertEqual(cabot_ui.i18n.localized_string("test", "en"), u"Test")
        self.assertEqual(cabot_ui.i18n.localized_string("test"), u"Test")

    def test_format(self):
        """test i18n format"""
        jaf = cabot_ui.i18n.localized_string("format", u'１', u'２', u'３', lang="ja")
        self.assertEqual(jaf, u'１足す２は３です')
        enf = cabot_ui.i18n.localized_string("format", 2, 3, 5)
        self.assertEqual(enf, "2 plus 3 equals to 5")

    def test_format_with_unicode(self):
        a = cabot_ui.i18n.localized_string("test", lang="ja")
        c = cabot_ui.i18n.localized_string("{}", a, lang="ja")
        self.assertEqual(a, c)

    def test_format_number(self):
        a = cabot_ui.i18n.localized_string("{}", 0, lang="ja")
        self.assertEqual(a, "0")
