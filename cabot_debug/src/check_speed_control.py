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
parser.add_option('-c', '--background_color', type=str, help='background color', default="yellow")
parser.add_option('-a', '--background_alpha', type=float, help='background_alpha', default=0.3)

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
    "/cabot/activity_log",
    "/current_floor",
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
        i = getIndex(topic, 3)
        data[i].append([st, t])
        data[i+1].append(msg.linear.x)
        data[i+2].append(msg.angular.z)
    elif topic in [
            "/cabot/touch",
            "/cabot/touch_raw",
            "/cabot/lidar_speed",
            "/cabot/people_speed",
            "/cabot/tf_speed",
            "/cabot/map_speed",
            "/cabot/user_speed",
            "/current_floor"]:
        i = getIndex(topic, 2)
        data[i].append([st, t])
        data[i+1].append(msg.data)
    elif topic in [
            "/cabot/activity_log"]:
        i = getIndex(topic, 2)
        # print(msg.text)
        if msg.text in [
                "navigation;event;navigation_start",
                "navigation;event;elevator_door_may_be_ready"]:
            data[i].append(st)
            data[i+1].append(1)
        elif msg.text in [
                "goal_canceled",
                "goal_completed"]:
            data[i].append(st)
            data[i+1].append(0)
            
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
line1, = ax1.plot([], [], 'red', linestyle='-', label='/cabot/cmd_vel.l')
line2, = ax1.plot([], [], 'blue', linestyle='--', label='/cabot/touch')
line3, = ax1.plot([], [], 'green', linestyle=':', label='/cabot/lidar_speed')
line4, = ax1.plot([], [], 'orange', linestyle=':', label='/cabot/people_speed')
line6, = ax1.plot([], [], 'brown', linestyle='-', label='/cabot/tf_speed')
line7, = ax1.plot([], [], 'pink', linestyle='-', label='/cabot/map_speed')
line8, = ax1.plot([], [], 'cyan', linestyle='-', label='/cabot/user_speed')
line9, = ax1.plot([], [], 'yellow', linestyle='-', label='/cmd_vel.l')
line10, = ax1.plot([], [], 'teal', linestyle='-', label='/cabot/cmd_vel.r')
line11, = ax1.plot([], [], 'magenta', linestyle='-', label='/cmd_vel.r')
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
line10.set_visible(False)
line11.set_visible(False)

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
    topic_index = []
    topic_index.extend([
        getIndex("/cabot/cmd_vel"),
        getIndex("/cabot/touch"),
        getIndex("/cabot/lidar_speed"),
        getIndex("/cabot/people_speed"),
        getIndex("/cabot/touch_raw"),
        getIndex("/cabot/tf_speed"),
        getIndex("/cabot/map_speed"),
        getIndex("/cabot/user_speed"),
        getIndex("/cmd_vel")
    ])
    line1.set_data([d[0] for d in data[topic_index[0]]], data[topic_index[0]+1])
    line2.set_data([d[0] for d in data[topic_index[1]]], data[topic_index[1]+1])
    line3.set_data([d[0] for d in data[topic_index[2]]], data[topic_index[2]+1])
    line4.set_data([d[0] for d in data[topic_index[3]]], data[topic_index[3]+1])
    line5.set_data([d[0] for d in data[topic_index[4]]], data[topic_index[4]+1])
    line6.set_data([d[0] for d in data[topic_index[5]]], data[topic_index[5]+1])
    line7.set_data([d[0] for d in data[topic_index[6]]], data[topic_index[6]+1])
    line8.set_data([d[0] for d in data[topic_index[7]]], data[topic_index[7]+1])
    line9.set_data([d[0] for d in data[topic_index[8]]], data[topic_index[8]+1])
    line10.set_data([d[0] for d in data[topic_index[0]]], data[topic_index[0]+2])
    line11.set_data([d[0] for d in data[topic_index[8]]], data[topic_index[8]+2])
    ax1.relim()
    ax1.autoscale_view()
    ax2.relim()
    ax2.autoscale_view()

    current_ticks = ax1.get_xticks()

    custom_labels = []
    for tick in current_ticks:
        closest_st = min([d[0] for d in data[topic_index[0]]], key=lambda x: abs(x - tick))
        t_value = next(d[1] for d in data[topic_index[0]] if d[0] == closest_st)
        custom_labels.append(f'{int(tick)}\nt={t_value:.2f}\nst=({closest_st:.2f})')

    ax1.set_xticks(current_ticks)
    ax1.set_xticklabels(custom_labels, ha='center')

    y1_min, y1_max = ax1.get_ylim()
    y2_min, y2_max = ax2.get_ylim()
    if y1_max > 2:
        y1_min = y1_min/(y1_max/2)
        ax1.set_ylim(bottom=y1_min)
        y1_max = 2
        ax1.set_ylim(top=y1_max)
    min_lim = y1_min*(y2_max/y1_max)
    ax2.set_ylim(bottom=min_lim)

    canvas.draw()

# Function to highlight specific time ranges based on the activity log events
def highlight_navigation_time(ax, data, index, color="yellow", alpha=0.3):
    start_time = None
    if len(data[index]) == 0:
        return
    if data[index+1][0] == 1:
        start_time = data[index][0]
    elif data[index+1][0] == 0:
        start_time = options.start

    for i in range(len(data[index])):
        if not start_time and data[index+1][i] == 1:
            start_time = data[index][i]
        elif start_time and data[index+1][i] == 0:
            end_time = data[index][i]
            ax.axvspan(start_time, end_time, color=color, alpha=alpha)
            start_time = None
            
    
    if start_time:
        x_max = data[getIndex("/cabot/cmd_vel")][-1][0]
        ax.axvspan(start_time, x_max, color=color, alpha=alpha)
    canvas.draw()

# Function to add vertical lines and labels based on the current floor data
def add_vertical_lines_and_labels(ax, time_data, value_data, color='red', visible=False):
    lines = []
    labels = []
    for i, time in enumerate(time_data):
        line = ax.axvline(x=time, color=color, linestyle='--', visible=visible)
        label = ax.text(time, ax.get_ylim()[1], f'{value_data[i]}', color=color, 
                        verticalalignment='bottom', horizontalalignment='center', visible=visible)
        lines.append(line)
        labels.append(label)
    return lines, labels

vertical_lines = []
vertical_labels = []
# Callback function to toggle the visibility of vertical lines and labels
def toggle_vertical_lines(var):
    global vertical_lines, vertical_labels
    visible = var.get()
    if not vertical_lines:
        index = getIndex("/current_floor")
        time_data = [d[0] for d in data[index]]
        value_data = data[index + 1]

        vertical_lines, vertical_labels = add_vertical_lines_and_labels(ax1, time_data, value_data, color='red', visible=visible)
    else:
        for line, label in zip(vertical_lines, vertical_labels):
            line.set_visible(var.get())
            label.set_visible(var.get())
    
    ax1.relim()
    ax1.autoscale_view()
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
var10 = tk.BooleanVar(value=False)
var11 = tk.BooleanVar(value=False)
var12 = tk.BooleanVar(value=True)
checkbox1 = tk.Checkbutton(cmd_vel_frame, text="Show /cabot/cmd_vel.l", variable=var1, command=lambda: toggle_line(line1, var1, ax1))
checkbox2 = tk.Checkbutton(touch_frame, text="Show /cabot/touch", variable=var2, command=lambda: toggle_line(line2, var2, ax1))
checkbox3 = tk.Checkbutton(speed_frame, text="Show /cabot/lidar_speed", variable=var3, command=lambda: toggle_line(line3, var3, ax1))
checkbox4 = tk.Checkbutton(speed_frame, text="Show /cabot/people_speed", variable=var4, command=lambda: toggle_line(line4, var4, ax1))
checkbox5 = tk.Checkbutton(touch_frame, text="Show /cabot/touch_raw", variable=var5, command=lambda: toggle_line(line5, var5, ax2))
checkbox6 = tk.Checkbutton(speed_frame, text="Show /cabot/tf_speed", variable=var6, command=lambda: toggle_line(line6, var6, ax1))
checkbox7 = tk.Checkbutton(speed_frame, text="Show /cabot/map_speed", variable=var7, command=lambda: toggle_line(line7, var7, ax1))
checkbox8 = tk.Checkbutton(speed_frame, text="Show /cabot/user_speed", variable=var8, command=lambda: toggle_line(line8, var8, ax1))
checkbox9 = tk.Checkbutton(cmd_vel_frame, text="Show /cmd_vel.l", variable=var9, command=lambda: toggle_line(line9, var9, ax1))
checkbox10 = tk.Checkbutton(cmd_vel_frame, text="Show /cabot/cmd_vel.r", variable=var10, command=lambda: toggle_line(line10, var10, ax1))
checkbox11 = tk.Checkbutton(cmd_vel_frame, text="Show /cmd_vel.r", variable=var11, command=lambda: toggle_line(line11, var11, ax1))
checkbox12 = tk.Checkbutton(frame, text=f"Show /current_floor", variable=var12, command=lambda: toggle_vertical_lines(var12))

# Create category checkboxes
cmd_vel_var = tk.BooleanVar(value=True)
speed_var = tk.BooleanVar(value=True)
touch_var = tk.BooleanVar(value=True)

cmd_vel_checkbox = tk.Checkbutton(cmd_vel_frame, text="all", variable=cmd_vel_var, command=lambda: toggle_category(cmd_vel_var, [(var1, line1), (var9, line9), (var10, line10), (var11, line11)]))
speed_checkbox = tk.Checkbutton(speed_frame, text="all", variable=speed_var, command=lambda: toggle_category(speed_var, [(var3, line3), (var4, line4), (var6, line6), (var7, line7), (var8, line8)]))
touch_checkbox = tk.Checkbutton(touch_frame, text="all", variable=touch_var, command=lambda: toggle_category(touch_var, [(var2, line2), (var5, line5)]))

# Arrange checkboxes in the frame
cmd_vel_checkbox.pack(side=tk.TOP, anchor='w')
checkbox1.pack(side=tk.TOP, anchor='w')
checkbox9.pack(side=tk.TOP, anchor='w')
checkbox10.pack(side=tk.TOP, anchor='w')
checkbox11.pack(side=tk.TOP, anchor='w')
speed_checkbox.pack(side=tk.TOP, anchor='w')
checkbox3.pack(side=tk.TOP, anchor='w')
checkbox4.pack(side=tk.TOP, anchor='w')
checkbox6.pack(side=tk.TOP, anchor='w')
checkbox7.pack(side=tk.TOP, anchor='w')
checkbox8.pack(side=tk.TOP, anchor='w')
touch_checkbox.pack(side=tk.TOP, anchor='w')
checkbox2.pack(side=tk.TOP, anchor='w')
checkbox5.pack(side=tk.TOP, anchor='w')

checkbox12.pack(side=tk.TOP, anchor='w')

# Plot data
plot_data()

highlight_navigation_time(ax2, data, getIndex("/cabot/activity_log"), color=options.background_color, alpha=options.background_alpha)

# Ensure vertical lines are displayed based on the initial checkbox state
toggle_vertical_lines(var12)

# Start the Tkinter main loop
root.mainloop()

