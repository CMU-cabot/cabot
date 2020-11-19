#!/usr/bin/env python

 ###############################################################################
 # Copyright (c) 2019  Carnegie Mellon University
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

import sys
import re
import rosbag
import numpy, numpy.linalg
import math
from matplotlib import pyplot as plt

bag = rosbag.Bag(sys.argv[1])

flag = False
up, down = 0, 0
for topic, msg, t in bag.read_messages(topics=["/cabot/event", "/move_base_simple/goal", "/cabot/poi"]):

    if topic == "/cabot/poi":
        for m in msg.markers:
            if m.text is not None:
                r = re.search("arrived", m.text)
                if r:
                    flag = False
    
    if topic == "/move_base_simple/goal":
        start = t
        flag = True


    if flag and topic == "/cabot/event":
        if msg.data == "click_4_1":
            up+=1
        if msg.data == "click_1_1":
            down+=1

print "%d,%d" % (up, down)
