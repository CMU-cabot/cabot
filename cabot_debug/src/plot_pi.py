#!/usr/bin/env python3

###############################################################################
#  Copyright (c) 2024  Miraikan
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

parser = OptionParser(usage="""
Example:
{0} -f <bag file>
""".format(sys.argv[0]))

parser.add_option('-f', '--file', type=str, help='bag file to be processed')

(options, args) = parser.parse_args()

if not options.file:
    parser.print_help()
    sys.exit(0)

bagfilename = options.file
reader = BagReader(bagfilename)

reader.set_filter_by_topics(["/PI_control",])
reader.set_filter_by_options(options)

time_stamps = []
control_errors = []

while reader.has_next():
    (topic, msg, t, st) = reader.serialize_next()
    try:
        time_stamps.append(st)
        control_errors.append(msg.error_spd_linear)
    except Exception as e:
        print(f"Failed to deserialize msg: {e}")

plt.figure(figsize=(10, 6))
plt.plot(time_stamps, control_errors, label='Control Error')
plt.xlabel('Time (s)')
plt.ylabel('Control Error')
plt.title('PI Control Error over Time')
plt.legend()
plt.grid(True)
plt.show()
