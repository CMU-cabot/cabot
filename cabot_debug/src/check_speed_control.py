#!/usr/bin/env python3

###############################################################################
# Copyright (c) 2019, 2024  Carnegie Mellon University and Miraikan
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
from optparse import OptionParser
from matplotlib import pyplot as plt
from cabot_common.rosbag2 import BagReader
import tkinter as tk
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

parser = OptionParser(usage="""
Example
{0} -f <bag file>                       # bagfile
""".format(sys.argv[0]))

parser.add_option('-f', '--file', type=str, help='bag file to be processed')
parser.add_option('-o', '--odom', action='store_true', help='output odom')
parser.add_option('-s', '--start', type=float, help='start time from the begining', default=0.0)
parser.add_option('-d', '--duration', type=float, help='duration from the start time', default=99999999999999)

(options, args) = parser.parse_args()

if not options.file:
    parser.print_help()
    sys.exit(0)

bagfilename = options.file
reader = BagReader(bagfilename)

reader.set_filter_by_topics([
    "/cabot/cmd_vel",
    "/cabot/touch",
    "/cabot/touch_raw",
    "/cabot/lidar_speed",
    "/cabot/people_speed",
    "/cabot/tf_speed",
    "/cabot/map_speed",
    "/cabot/user_speed",
    "/cmd_vel",
])
reader.set_filter_by_options(options)  # filter by start and duration

data = tuple([[] for _ in range(100)])
indexes = {}
index = 0

def getIndex(name, increment=0):
    global indexes, index
    if name not in indexes:
        indexes[name] = index
        index += increment
    return indexes[name]

while reader.has_next():
    (topic, msg, t, st) = reader.serialize_next()
    if not topic:
        continue

    if topic in [ 
            "/cabot/cmd_vel",
            "/cmd_vel"]:
        i = getIndex(topic, 2)
        data[i].append(st)
        data[i+1].append(msg.linear.x)
    elif topic in [
            "/cabot/touch",
            "/cabot/touch_raw",
            "/cabot/lidar_speed",
            "/cabot/people_speed",
            "/cabot/tf_speed",
            "/cabot/map_speed",
            "/cabot/user_speed"]:
        i = getIndex(topic, 2)
        data[i].append(st)
        data[i+1].append(msg.data)

# Create a Tkinter window
root = tk.Tk()
root.title("Matplotlib with Tkinter")

# Create a frame to display checkboxes on the left side
frame = tk.Frame(root)
frame.pack(side=tk.LEFT, fill=tk.Y)

# Create frames for each category
cmd_vel_frame = tk.LabelFrame(frame, text="cmd_vel")
cmd_vel_frame.pack(side=tk.TOP, fill=tk.X, padx=5, pady=5)
speed_frame = tk.LabelFrame(frame, text="speed")
speed_frame.pack(side=tk.TOP, fill=tk.X, padx=5, pady=5)
touch_frame = tk.LabelFrame(frame, text="touch")
touch_frame.pack(side=tk.TOP, fill=tk.X, padx=5, pady=5)

# Create a Matlotlib figure
fig, ax1 = plt.subplots(figsize=(20, 10))
line1, = ax1.plot([], [], 'red', linestyle='-', label='/cabot/cmd_vel')
line2, = ax1.plot([], [], 'blue', linestyle='--', label='/cabot/touch')
line3, = ax1.plot([], [], 'green', linestyle=':', label='/cabot/lidar_speed')
line4, = ax1.plot([], [], 'orange', linestyle=':', label='/cabot/people_speed')
line6, = ax1.plot([], [], 'brown', linestyle='-', label='/cabot/tf_speed')
line7, = ax1.plot([], [], 'pink', linestyle='-', label='/cabot/map_speed')
line8, = ax1.plot([], [], 'cyan', linestyle='-', label='/cabot/user_speed')
line9, = ax1.plot([], [], 'yellow', linestyle='-', label='/cmd_vel')
ax1.legend(bbox_to_anchor=(1.00, 1), loc='upper left')

ax2 = ax1.twinx()
line5, = ax2.plot([], [], 'purple', linestyle='-', label='/cabot/touch_raw')
ax2.legend(bbox_to_anchor=(1.00, 1), loc='center left')

# Initially set to invisible
line5.set_visible(False)
line6.set_visible(False)
line7.set_visible(False)
line8.set_visible(False)
line9.set_visible(False)

# Embed the Matplotlib figure into the Tkinter window using FigureCanvasTkAgg
canvas = FigureCanvasTkAgg(fig, master=root)
canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=1)
canvas.draw()

# Callback function for checkboxes
def toggle_line(line, var, ax=None):
    line.set_visible(var.get())
    if ax:
        ax.relim()
        ax.autoscale_view()
    canvas.draw()

# Callback function for category checkboxes
def toggle_category(var, checkboxes):
    for checkbox, line in checkboxes:
        checkbox.set(var.get())
        toggle_line(line, var)

# Plot data function
def plot_data():
    line1.set_data(data[getIndex("/cabot/cmd_vel")], data[getIndex("/cabot/cmd_vel")+1])
    line2.set_data(data[getIndex("/cabot/touch")], data[getIndex("/cabot/touch")+1])
    line3.set_data(data[getIndex("/cabot/lidar_speed")], data[getIndex("/cabot/lidar_speed")+1])
    line4.set_data(data[getIndex("/cabot/people_speed")], data[getIndex("/cabot/people_speed")+1])
    line5.set_data(data[getIndex("/cabot/touch_raw")], data[getIndex("/cabot/touch_raw")+1])
    line6.set_data(data[getIndex("/cabot/tf_speed")], data[getIndex("/cabot/tf_speed")+1])
    line7.set_data(data[getIndex("/cabot/map_speed")], data[getIndex("/cabot/map_speed")+1])
    line8.set_data(data[getIndex("/cabot/user_speed")], data[getIndex("/cabot/user_speed")+1])
    line9.set_data(data[getIndex("/cmd_vel")], data[getIndex("/cmd_vel")+1])
    ax1.relim()
    ax1.autoscale_view()
    ax2.relim()
    ax2.autoscale_view()

    y1_min, y1_max = ax1.get_ylim()
    y2_min, y2_max = ax2.get_ylim()
    min_lim = y1_min*(y2_max/y1_max)
    ax2.set_ylim(bottom=min_lim)

    canvas.draw()

# Create individual checkboxes
var1 = tk.BooleanVar(value=True)
var2 = tk.BooleanVar(value=True)
var3 = tk.BooleanVar(value=True)
var4 = tk.BooleanVar(value=True)
var5 = tk.BooleanVar(value=False)
var6 = tk.BooleanVar(value=False)
var7 = tk.BooleanVar(value=False)
var8 = tk.BooleanVar(value=False)
var9 = tk.BooleanVar(value=False)
checkbox1 = tk.Checkbutton(cmd_vel_frame, text="Show /cabot/cmd_vel", variable=var1, command=lambda: toggle_line(line1, var1, ax1))
checkbox2 = tk.Checkbutton(touch_frame, text="Show /cabot/touch", variable=var2, command=lambda: toggle_line(line2, var2, ax1))
checkbox3 = tk.Checkbutton(speed_frame, text="Show /cabot/lidar_speed", variable=var3, command=lambda: toggle_line(line3, var3, ax1))
checkbox4 = tk.Checkbutton(speed_frame, text="Show /cabot/people_speed", variable=var4, command=lambda: toggle_line(line4, var4, ax1))
checkbox5 = tk.Checkbutton(touch_frame, text="Show /cabot/touch_raw", variable=var5, command=lambda: toggle_line(line5, var5, ax2))
checkbox6 = tk.Checkbutton(speed_frame, text="Show /cabot/tf_speed", variable=var6, command=lambda: toggle_line(line6, var6, ax1))
checkbox7 = tk.Checkbutton(speed_frame, text="Show /cabot/map_speed", variable=var7, command=lambda: toggle_line(line7, var7, ax1))
checkbox8 = tk.Checkbutton(speed_frame, text="Show /cabot/user_speed", variable=var8, command=lambda: toggle_line(line8, var8, ax1))
checkbox9 = tk.Checkbutton(cmd_vel_frame, text="Show /cmd_vel", variable=var9, command=lambda: toggle_line(line9, var9, ax1))

# Create category checkboxes
cmd_vel_var = tk.BooleanVar(value=True)
speed_var = tk.BooleanVar(value=True)
touch_var = tk.BooleanVar(value=True)

cmd_vel_checkbox = tk.Checkbutton(cmd_vel_frame, text="all", variable=cmd_vel_var, command=lambda: toggle_category(cmd_vel_var, [(var1, line1), (var9, line9)]))
speed_checkbox = tk.Checkbutton(speed_frame, text="all", variable=speed_var, command=lambda: toggle_category(speed_var, [(var3, line3), (var4, line4), (var6, line6), (var7, line7), (var8, line8)]))
touch_checkbox = tk.Checkbutton(touch_frame, text="all", variable=touch_var, command=lambda: toggle_category(touch_var, [(var2, line2), (var5, line5)]))

# Arrange checkboxes in the frame
cmd_vel_checkbox.pack(side=tk.TOP, anchor='w')
checkbox1.pack(side=tk.TOP, anchor='w')
checkbox9.pack(side=tk.TOP, anchor='w')
speed_checkbox.pack(side=tk.TOP, anchor='w')
checkbox3.pack(side=tk.TOP, anchor='w')
checkbox4.pack(side=tk.TOP, anchor='w')
checkbox6.pack(side=tk.TOP, anchor='w')
checkbox7.pack(side=tk.TOP, anchor='w')
checkbox8.pack(side=tk.TOP, anchor='w')
touch_checkbox.pack(side=tk.TOP, anchor='w')
checkbox2.pack(side=tk.TOP, anchor='w')
checkbox5.pack(side=tk.TOP, anchor='w')

# Plot data
plot_data()

# Start the Tkinter main loop
root.mainloop()

