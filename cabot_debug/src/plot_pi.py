#!/usr/bin/env python3

###############################################################################
#  Copyright (c) 2024  Carnegie Mellon University and Miraikan
#
#  Permission is hereby granted, free of charge, to any person obtaining a copy
#  of this software and associated documentation files (the "Software"), to deal
#  in the Software without restriction, including without limitation the rights
#  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
#  copies of the Software, and to permit persons to whom the Software is
#  furnished to do so, subject to the following conditions:
# 
#  The above copyright notice and this permission notice shall be included in
#  all copies or substantial portions of the Software.
# 
#  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
#  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
#  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
#  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
#  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
#  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
#  THE SOFTWARE.
###############################################################################

import sys
from optparse import OptionParser
import matplotlib.pyplot as plt
from cabot_common.rosbag2 import BagReader
from matplotlib.widgets import CheckButtons

parser = OptionParser(usage="""
Example:
{0} -f <bag file>
""".format(sys.argv[0]))

parser.add_option('-f', '--file', type=str, help='bag file to be processed')
parser.add_option('-s', '--start', type=float, help='start time from the begining', default=0.0)
parser.add_option('-d', '--duration', type=float, help='duration from the start time', default=99999999999999)

(options, args) = parser.parse_args()

if not options.file:
    parser.print_help()
    sys.exit(0)

bagfilename = options.file
reader = BagReader(bagfilename)

reader.set_filter_by_topics(["/PI_control",])
reader.set_filter_by_options(options)

time_stamps = []
control_errors = {
    "current_spd_linear": [],
    "measured_spd_linear": [],
    "error_spd_linear": [],
    "integral_linear": [],
    "target_spd_turn": [],
    "measured_spd_turn": [],
    "error_spd_turn": [],
    "integral_turn": []
}

while reader.has_next():
    (topic, msg, t, st) = reader.serialize_next()
    try:
        time_stamps.append(st)
        control_errors["current_spd_linear"].append(msg.current_spd_linear)
        control_errors["measured_spd_linear"].append(msg.measured_spd_linear)
        control_errors["error_spd_linear"].append(msg.error_spd_linear)
        control_errors["integral_linear"].append(msg.integral_linear)
        control_errors["target_spd_turn"].append(msg.target_spd_turn)
        control_errors["measured_spd_turn"].append(msg.measured_spd_turn)
        control_errors["error_spd_turn"].append(msg.error_spd_turn)
        control_errors["integral_turn"].append(msg.integral_turn)
    except Exception as e:
        print(f"Failed to deserialize msg: {e}")

fig, ax = plt.subplots(figsize=(10, 6))
lines = {
    "current_spd_linear": ax.plot(time_stamps, control_errors["current_spd_linear"], label='Current Speed Linear')[0],
    "measured_spd_linear": ax.plot(time_stamps, control_errors["measured_spd_linear"], label='Measured Speed Linear')[0],
    "error_spd_linear": ax.plot(time_stamps, control_errors["error_spd_linear"], label='Error Speed Linear')[0],
    "integral_linear": ax.plot(time_stamps, control_errors["integral_linear"], label='Integral Linear')[0],
    "target_spd_turn": ax.plot(time_stamps, control_errors["target_spd_turn"], label='Target Speed Turn')[0],
    "measured_spd_turn": ax.plot(time_stamps, control_errors["measured_spd_turn"], label='Measured Speed Turn')[0],
    "error_spd_turn": ax.plot(time_stamps, control_errors["error_spd_turn"], label='Error Speed Turn')[0],
    "integral_turn": ax.plot(time_stamps, control_errors["integral_turn"], label='Integral Turn')[0]
}

plt.xlabel('Time (s)')
plt.ylabel('Control Error value')
plt.title('PI Control Data over Time')
plt.grid(True)

rax = plt.axes([0.9, 0.4, 0.15, 0.4], facecolor='lightgoldenrodyellow')
labels = list(lines.keys())
visibility = [line.get_visible() for line in lines.values()]
check = CheckButtons(rax, labels, visibility)

def func(label):
    line = lines[label]
    line.set_visible(not line.get_visible())
    plt.draw()

check.on_clicked(func)
plt.show()
