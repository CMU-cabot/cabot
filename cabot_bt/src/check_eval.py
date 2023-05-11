#!/usr/bin/python3

# Copyright (c) 2023  Carnegie Mellon University
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
from optparse import OptionParser
from cabot_common.rosbag2 import BagReader


parser = OptionParser(usage="""
Example
{0} -f <bag file>                        # analyze DWB critics evaluations
""".format(sys.argv[0]))

parser.add_option('-f', '--file', type=str, help='bag file to analyze')
parser.add_option('-s', '--start', type=float, help='start time from the begining', default=0.0)
parser.add_option('-d', '--duration', type=float, help='duration from the start time', default=99999999999999)
parser.add_option('-n', '--namespace', type=str, help='not specified or /local', default='')
parser.add_option('-i', '--items', type=int, help='number of items to print', default=5)

(options, args) = parser.parse_args()

if not options.file:
    parser.print_help()
    sys.exit(0)

bagfilename = options.file
reader = BagReader(bagfilename)

eval_topic = options.namespace+"/evaluation"

reader.set_filter_by_topics([
    eval_topic,
    "/odom",
])
reader.set_filter_by_options(options)  # filter by start and duration


items = options.items
total = 0
cmd = 0
vel = 0


while reader.has_next():
    (topic, msg, time, st) = reader.serialize_next()

    if topic == "/odom":
        vel = msg.twist.twist.linear.x
    elif topic == eval_topic:
        if len(msg.twists) == 0:
            print(F"[{time}] - error no twists")
            continue

        twist = msg.twists[msg.best_index]
        cmd = twist.traj.velocity.x
        total = twist.total

        index = msg.best_index

        print(F"[{time:18.9f}, {st:5.2f}] cmd={cmd:8.4f}, vel={vel:8.4f}")
        print(F"[{index:4d}] total={twist.total:10.4f} ({twist.traj.velocity.x:4.2f}, {twist.traj.velocity.theta:4.2f})")
        for score in twist.scores:
            print(F"  {score.name:20s} {score.raw_score:10.4f} * {score.scale:10.4f} = {score.raw_score*score.scale:10.4f}")

        msg.twists.sort(key=lambda x: x.total)
        for i, twist in enumerate(msg.twists):
            if items < i:
                break
            print(F"[{i:4d}] total={twist.total:10.4f} ({twist.traj.velocity.x:4.2f}, {twist.traj.velocity.theta:4.2f})")
            for score in twist.scores:
                print(F"  {score.name:20s} {score.raw_score:10.4f} * {score.scale:10.4f} = {score.raw_score*score.scale:10.4f}")
