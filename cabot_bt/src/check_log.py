#!/usr/bin/python3

# Copyright (c) 2020, 2023  Carnegie Mellon University
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


import sys
from rclpy.time import Time
from optparse import OptionParser
from cabot_common.rosbag2 import BagReader


parser = OptionParser(usage="""
Example
{0} -f <bag file>                        # analyze BT tree log
""".format(sys.argv[0]))

parser.add_option('-f', '--file', type=str, help='bag file to analyze')
parser.add_option('-s', '--start', type=float, help='start time from the begining', default=0.0)
parser.add_option('-d', '--duration', type=float, help='duration from the start time', default=99999999999999)

(options, args) = parser.parse_args()

if not options.file:
    parser.print_help()
    sys.exit(0)

bagfilename = options.file
reader = BagReader(bagfilename)

reader.set_filter_by_topics([
    "/behavior_tree_log",
])
reader.set_filter_by_options(options)  # filter by start and duration

log_counter = 0

prev = None

stack = [None]

while reader.has_next():
    (topic, msg, time, st) = reader.serialize_next()

    for i, e in enumerate(msg.event_log):
        log_counter += 1
        print("[%.2f]%s%s (%s-%s)" % (
            Time.from_msg(e.timestamp).nanoseconds/1e9,
            " "*len(stack),
            e.node_name,
            e.previous_status, e.
            current_status
        ))
