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
from matplotlib.ticker import AutoLocator, FixedLocator
from cabot_common.rosbag2 import BagReader
import tkinter as tk
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import subprocess
from bisect import bisect_left, bisect_right

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
bag_info_reader = BagReader(bagfilename)
BAG_DURATION = bag_info_reader.bag_duration()

BAG_TOPICS = [
    "/cabot/cmd_vel",
    "/cabot/touch",
    "/cabot/touch_raw",
    "/cabot/lidar_speed",
    "/cabot/people_speed",
    "/cabot/tf_speed",
    "/cabot/map_speed",
    "/cabot/low_lidar_speed",
    "/cabot/wheelie_speed",
    "/cabot/social_distance_speed",
    "/cabot/pure_velocity_obstacle_speed",
    "/cabot/combined_speed",
    "/cmd_vel",
    "/cabot/activity_log",
    "/current_floor",
    "/cabot/capacitive/touch",
    "/cabot/capacitive/touch_raw",
    "/cabot/tof/touch",
    "/cabot/tof/touch_raw",
]

TOPIC_SPECS = [
    ("/cabot/cmd_vel", 3),
    ("/cmd_vel", 3),
    ("/cabot/touch", 2),
    ("/cabot/touch_raw", 2),
    ("/cabot/lidar_speed", 2),
    ("/cabot/people_speed", 2),
    ("/cabot/tf_speed", 2),
    ("/cabot/map_speed", 2),
    ("/cabot/low_lidar_speed", 2),
    ("/cabot/wheelie_speed", 2),
    ("/cabot/social_distance_speed", 2),
    ("/cabot/pure_velocity_obstacle_speed", 2),
    ("/cabot/combined_speed", 2),
    ("/cabot/activity_log", 2),
    ("/current_floor", 2),
    ("/cabot/capacitive/touch", 2),
    ("/cabot/capacitive/touch_raw", 2),
    ("/cabot/tof/touch", 2),
    ("/cabot/tof/touch_raw", 2),
    ("/cabot/user_speed", 2),
]

TOPIC_INDEX = {}
slot_count = 0
for topic_name, width in TOPIC_SPECS:
    TOPIC_INDEX[topic_name] = slot_count
    slot_count += width

CMD_VEL_TOPICS = {"/cabot/cmd_vel", "/cmd_vel"}
VALUE_TOPICS = {
    "/cabot/touch",
    "/cabot/touch_raw",
    "/cabot/lidar_speed",
    "/cabot/people_speed",
    "/cabot/tf_speed",
    "/cabot/map_speed",
    "/cabot/low_lidar_speed",
    "/cabot/wheelie_speed",
    "/current_floor",
    "/cabot/social_distance_speed",
    "/cabot/pure_velocity_obstacle_speed",
    "/cabot/combined_speed",
    "/cabot/capacitive/touch",
    "/cabot/capacitive/touch_raw",
    "/cabot/tof/touch",
    "/cabot/tof/touch_raw",
}
ACTIVITY_LOG_TOPIC = "/cabot/activity_log"
USER_SPEED_TOPIC = "/cabot/user_speed"


def getIndex(name):
    return TOPIC_INDEX[name]


def init_data():
    return tuple([[] for _ in range(slot_count)])


def create_reader():
    reader = BagReader(bagfilename)
    reader.set_filter_by_topics(BAG_TOPICS)
    return reader


def get_user_speed():
    process = subprocess.Popen(
        ["ros2", "run", "cabot_debug", "print_topics.py", "-f", bagfilename, "-t", USER_SPEED_TOPIC],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True
    )

    st = []
    data = []

    for line in process.stdout:
        line = line.strip()
        try:
            st.append(float(line.split()[-2].split("(")[1].rstrip("):")))
        except:
            print(f"ERROR: {line}")
            continue
        data.append(float(line.split()[-1]))

    process.wait()
    return st, data


def build_user_speed_range(raw_st, raw_data, start, duration, end_time):
    process_st = []
    process_data = []

    tmp_st = start
    tmp_data = 1.0
    range_end = min(float(start + duration), float(end_time))

    for a, b in zip(raw_st, raw_data):
        if start > a:
            tmp_data = b
            continue

        process_st.append(tmp_st)
        process_data.append(tmp_data)

        if range_end < a:
            break

        tmp_st = a
        process_st.append(tmp_st)
        process_data.append(tmp_data)
        tmp_data = b

    process_st.append(tmp_st)
    process_data.append(tmp_data)
    process_st.append(range_end)
    process_data.append(tmp_data)

    return process_st, process_data


def load_data():
    data = init_data()
    reader = create_reader()

    while reader.has_next():
        (topic, msg, t, st) = reader.serialize_next()
        if not topic:
            continue

        if topic in CMD_VEL_TOPICS:
            i = getIndex(topic)
            data[i].append([st, t])
            data[i+1].append(msg.linear.x)
            data[i+2].append(msg.angular.z)
        elif topic in VALUE_TOPICS:
            i = getIndex(topic)
            data[i].append([st, t])
            data[i+1].append(msg.data)
        elif topic == ACTIVITY_LOG_TOPIC:
            i = getIndex(topic)
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

    raw_us_st, raw_us_data = get_user_speed()
    us_st, us_data = build_user_speed_range(raw_us_st, raw_us_data, 0.0, BAG_DURATION, BAG_DURATION)
    i = getIndex(USER_SPEED_TOPIC)
    data[i].extend(us_st)
    data[i+1].extend(us_data)
    return data


current_start = options.start
current_duration = options.duration
full_data = load_data()
data = init_data()

# Create a Tkinter window
root = tk.Tk()
root.title("check_speed_control plot")

control_window = tk.Toplevel(root)
control_window.title("check_speed_control controls")
control_window.geometry("360x1000")

control_canvas = tk.Canvas(control_window, highlightthickness=0)
control_scrollbar = tk.Scrollbar(control_window, orient=tk.VERTICAL, command=control_canvas.yview)
control_canvas.configure(yscrollcommand=control_scrollbar.set)
control_scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
control_canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

frame = tk.Frame(control_canvas)
control_canvas_window = control_canvas.create_window((0, 0), window=frame, anchor="nw")


def update_control_scroll_region(event=None):
    control_canvas.configure(scrollregion=control_canvas.bbox("all"))


def update_control_width(event):
    control_canvas.itemconfigure(control_canvas_window, width=event.width)


frame.bind("<Configure>", update_control_scroll_region)
control_canvas.bind("<Configure>", update_control_width)

range_frame = tk.LabelFrame(frame, text="range")
range_frame.pack(side=tk.TOP, fill=tk.X, padx=5, pady=5)

# Create frames for each category
cmd_vel_frame = tk.LabelFrame(frame, text="cmd_vel")
cmd_vel_frame.pack(side=tk.TOP, fill=tk.X, padx=5, pady=5)
speed_frame = tk.LabelFrame(frame, text="speed")
speed_frame.pack(side=tk.TOP, fill=tk.X, padx=5, pady=5)
touch_frame = tk.LabelFrame(frame, text="touch")
touch_frame.pack(side=tk.TOP, fill=tk.X, padx=5, pady=5)

# Create a Matlotlib figure
fig, ax1 = plt.subplots(figsize=(20, 10))
fig.subplots_adjust(left=0.25, right=0.90, top=0.95)
line1, = ax1.plot([], [], 'red', linestyle='-', label='/cabot/cmd_vel.l')
line2, = ax1.plot([], [], 'blue', linestyle='-', label='/cabot/touch')
line3, = ax1.plot([], [], 'green', linestyle=':', label='/cabot/lidar_speed')
line4, = ax1.plot([], [], 'orange', linestyle=':', label='/cabot/people_speed')
line6, = ax1.plot([], [], 'brown', linestyle='-', label='/cabot/tf_speed')
line7, = ax1.plot([], [], 'pink', linestyle='-', label='/cabot/map_speed')
line8, = ax1.plot([], [], 'cyan', linestyle='-', label='/cabot/user_speed')
line9, = ax1.plot([], [], 'yellow', linestyle='-', label='/cmd_vel.l')
line10, = ax1.plot([], [], 'teal', linestyle='-', label='/cabot/cmd_vel.r')
line11, = ax1.plot([], [], 'magenta', linestyle='-', label='/cmd_vel.r')
line12, = ax1.plot([], [], 'purple', linestyle='--', label='/cabot/social_distance_speed')
line13, = ax1.plot([], [], 'lime', linestyle=':', label='/cabot/pure_velocity_obstacle_speed')
line14, = ax1.plot([], [], 'gray', linestyle=':', label='/cabot/combined_speed')
line15, = ax1.plot([], [], 'black', linestyle='-', label='/cabot/capacitive/touch')
line17, = ax1.plot([], [], 'gold', linestyle='-', label='/cabot/tof/touch')
line19, = ax1.plot([], [], 'purple', linestyle='--', label='/cabot/low_lidar_speed')
line20, = ax1.plot([], [], 'lime', linestyle=':', label='/cabot/wheelie_speed')

ax2 = ax1.twinx()
line5, = ax2.plot([], [], 'navy', linestyle='--', label='/cabot/touch_raw')
line16, = ax2.plot([], [], 'navy', linestyle='--', label='/cabot/capacitive/touch_raw')
ax2.tick_params(axis='y', colors='navy')

ax3 = ax1.twinx()
ax3.spines["right"].set_position(("axes", 1.03))
line18, = ax3.plot([], [], 'maroon', linestyle='--', label='/cabot/tof/touch_raw')
ax3.tick_params(axis='y', colors='maroon')

lines1, labels1 = ax1.get_legend_handles_labels()
lines2, labels2 = ax2.get_legend_handles_labels()
lines3, labels3 = ax3.get_legend_handles_labels()
ax1.legend(
    lines1 + lines2 + lines3,
    labels1 + labels2 + labels3,
    loc="upper left",
    bbox_to_anchor=(-0.26, 1.0),
    borderaxespad=0.0,
    ncol=1,
    fontsize="small"
)

# Initially set to invisible
line5.set_visible(False)
line6.set_visible(False)
line7.set_visible(False)
line8.set_visible(False)
line9.set_visible(False)
line10.set_visible(False)
line11.set_visible(False)
line12.set_visible(False)
line13.set_visible(False)
line14.set_visible(False)
line15.set_visible(False)
line16.set_visible(False)
line17.set_visible(False)
line18.set_visible(False)
line19.set_visible(False)
line20.set_visible(False)

# Embed the Matplotlib figure into the Tkinter window using FigureCanvasTkAgg
canvas = FigureCanvasTkAgg(fig, master=root)
canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=1)
canvas.draw()

start_var = tk.StringVar(value=f"{options.start:g}")
duration_var = tk.StringVar(value=f"{options.duration:g}")
status_var = tk.StringVar(value="")
navigation_spans = []

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

def get_time_pairs(topic):
    return data[getIndex(topic)]


def get_series(topic, offset=1):
    return data[getIndex(topic) + offset]


def get_selected_end():
    end_time = min(current_start + current_duration, BAG_DURATION)
    if end_time <= current_start:
        end_time = current_start + 0.001
    return end_time


def clear_navigation_highlights():
    global navigation_spans
    for span in navigation_spans:
        span.remove()
    navigation_spans = []


def slice_topic_data(topic, start, end):
    full_times = full_data[getIndex(topic)]
    full_values = full_data[getIndex(topic) + 1]

    if not full_times:
        return [], []

    st_values = [item[0] for item in full_times]
    left = bisect_left(st_values, start)
    right = bisect_right(st_values, end)
    return full_times[left:right], full_values[left:right]


def slice_event_data(topic, start, end):
    full_times = full_data[getIndex(topic)]
    full_values = full_data[getIndex(topic) + 1]

    if not full_times:
        return [], []

    left = bisect_left(full_times, start)
    right = bisect_right(full_times, end)
    return full_times[left:right], full_values[left:right]


def slice_cmd_vel_data(topic, start, end):
    full_times = full_data[getIndex(topic)]
    linear_values = full_data[getIndex(topic) + 1]
    angular_values = full_data[getIndex(topic) + 2]

    if not full_times:
        return [], [], []

    st_values = [item[0] for item in full_times]
    left = bisect_left(st_values, start)
    right = bisect_right(st_values, end)
    return full_times[left:right], linear_values[left:right], angular_values[left:right]


def slice_user_speed_data(start, duration):
    full_times = full_data[getIndex(USER_SPEED_TOPIC)]
    full_values = full_data[getIndex(USER_SPEED_TOPIC) + 1]
    end = min(start + duration, BAG_DURATION)

    if not full_times:
        return [], []

    left = bisect_right(full_times, start)
    right = bisect_left(full_times, end)
    current_value_index = max(left - 1, 0)

    sliced_times = [start]
    sliced_values = [full_values[current_value_index]]

    sliced_times.extend(full_times[left:right])
    sliced_values.extend(full_values[left:right])

    end_value_index = max(bisect_right(full_times, end) - 1, 0)
    sliced_times.append(end)
    sliced_values.append(full_values[end_value_index])
    return sliced_times, sliced_values


def update_data_cache(start, duration):
    global data
    end = min(start + duration, BAG_DURATION)
    data = init_data()

    for topic in CMD_VEL_TOPICS:
        times, linear_values, angular_values = slice_cmd_vel_data(topic, start, end)
        i = getIndex(topic)
        data[i].extend(times)
        data[i+1].extend(linear_values)
        data[i+2].extend(angular_values)

    for topic in VALUE_TOPICS:
        times, values = slice_topic_data(topic, start, end)
        i = getIndex(topic)
        data[i].extend(times)
        data[i+1].extend(values)

    times, values = slice_event_data(ACTIVITY_LOG_TOPIC, start, end)
    i = getIndex(ACTIVITY_LOG_TOPIC)
    data[i].extend(times)
    data[i+1].extend(values)

    times, values = slice_user_speed_data(start, duration)
    i = getIndex(USER_SPEED_TOPIC)
    data[i].extend(times)
    data[i+1].extend(values)

# Plot data function
def plot_data():
    line1.set_data([d[0] for d in get_time_pairs("/cabot/cmd_vel")], get_series("/cabot/cmd_vel"))
    line2.set_data([d[0] for d in get_time_pairs("/cabot/touch")], get_series("/cabot/touch"))
    line3.set_data([d[0] for d in get_time_pairs("/cabot/lidar_speed")], get_series("/cabot/lidar_speed"))
    line4.set_data([d[0] for d in get_time_pairs("/cabot/people_speed")], get_series("/cabot/people_speed"))
    line5.set_data([d[0] for d in get_time_pairs("/cabot/touch_raw")], get_series("/cabot/touch_raw"))
    line6.set_data([d[0] for d in get_time_pairs("/cabot/tf_speed")], get_series("/cabot/tf_speed"))
    line7.set_data([d[0] for d in get_time_pairs("/cabot/map_speed")], get_series("/cabot/map_speed"))
    line8.set_data(data[getIndex(USER_SPEED_TOPIC)], get_series(USER_SPEED_TOPIC))
    line9.set_data([d[0] for d in get_time_pairs("/cmd_vel")], get_series("/cmd_vel"))
    line10.set_data([d[0] for d in get_time_pairs("/cabot/cmd_vel")], get_series("/cabot/cmd_vel", 2))
    line11.set_data([d[0] for d in get_time_pairs("/cmd_vel")], get_series("/cmd_vel", 2))
    line12.set_data([d[0] for d in get_time_pairs("/cabot/social_distance_speed")], get_series("/cabot/social_distance_speed"))
    line13.set_data([d[0] for d in get_time_pairs("/cabot/pure_velocity_obstacle_speed")], get_series("/cabot/pure_velocity_obstacle_speed"))
    line14.set_data([d[0] for d in get_time_pairs("/cabot/combined_speed")], get_series("/cabot/combined_speed"))
    line19.set_data([d[0] for d in get_time_pairs("/cabot/low_lidar_speed")], get_series("/cabot/low_lidar_speed"))
    line20.set_data([d[0] for d in get_time_pairs("/cabot/wheelie_speed")], get_series("/cabot/wheelie_speed"))
    line15.set_data([d[0] for d in get_time_pairs("/cabot/capacitive/touch")], get_series("/cabot/capacitive/touch"))
    line16.set_data([d[0] for d in get_time_pairs("/cabot/capacitive/touch_raw")], get_series("/cabot/capacitive/touch_raw"))
    line17.set_data([d[0] for d in get_time_pairs("/cabot/tof/touch")], get_series("/cabot/tof/touch"))
    line18.set_data([d[0] for d in get_time_pairs("/cabot/tof/touch_raw")], get_series("/cabot/tof/touch_raw"))
    ax1.relim()
    ax1.autoscale_view()
    ax2.relim()
    ax2.autoscale_view()
    ax3.relim()
    ax3.autoscale_view()
    range_end = get_selected_end()
    ax1.set_xlim(current_start, range_end)

    locator = AutoLocator()
    current_ticks = [
        tick for tick in locator.tick_values(current_start, range_end)
        if current_start <= tick <= range_end
    ]
    if len(current_ticks) < 2:
        current_ticks = [current_start, range_end]
    ax1.xaxis.set_major_locator(FixedLocator(current_ticks))
    cmd_vel_data = get_time_pairs("/cabot/cmd_vel")

    if cmd_vel_data:
        custom_labels = []
        for tick in current_ticks:
            closest_data = min(cmd_vel_data, key=lambda x: abs(x[0] - tick))
            custom_labels.append(f'{int(tick)}\nt={closest_data[1]:.2f}\nst=({closest_data[0]:.2f})')

        ax1.set_xticklabels(custom_labels, ha='center')
    else:
        ax1.set_xticklabels([f"{tick:.2f}" for tick in current_ticks], ha='center')

    y1_min, y1_max = ax1.get_ylim()
    y2_min, y2_max = ax2.get_ylim()
    y3_min, y3_max = ax3.get_ylim()
    if y1_max <= 0:
        ax1.set_ylim(bottom=0, top=2)
        y1_min, y1_max = ax1.get_ylim()
    elif y1_max > 2:
        y1_min = y1_min/(y1_max/2)
        ax1.set_ylim(bottom=y1_min)
        y1_max = 2
        ax1.set_ylim(top=y1_max)
    if y2_max > 0:
        y2_min_lim = y1_min*(y2_max/y1_max)
        ax2.set_ylim(bottom=y2_min_lim)
    if y3_max > 0:
        y3_min_lim = y1_min*(y3_max/y1_max)
        ax3.set_ylim(bottom=y3_min_lim)

    canvas.draw()

# Function to highlight specific time ranges based on the activity log events
def highlight_navigation_time(ax, data, index, default_start, color="yellow", alpha=0.3):
    global navigation_spans
    start_time = None
    if len(data[index]) == 0:
        return
    if data[index+1][0] == 1:
        start_time = data[index][0]
    elif data[index+1][0] == 0:
        start_time = default_start

    for i in range(len(data[index])):
        if start_time is None and data[index+1][i] == 1:
            start_time = data[index][i]
        elif start_time is not None and data[index+1][i] == 0:
            end_time = data[index][i]
            navigation_spans.append(ax.axvspan(start_time, end_time, color=color, alpha=alpha))
            start_time = None
            
    
    if start_time is not None:
        cmd_vel_data = get_time_pairs("/cabot/cmd_vel")
        x_max = cmd_vel_data[-1][0] if cmd_vel_data else get_selected_end()
        navigation_spans.append(ax.axvspan(start_time, x_max, color=color, alpha=alpha))

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


def clear_vertical_lines():
    global vertical_lines, vertical_labels
    for line in vertical_lines:
        line.remove()
    for label in vertical_labels:
        label.remove()
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
    ax1.set_xlim(current_start, get_selected_end())
    canvas.draw()


def parse_range_values():
    start_text = start_var.get().strip()
    duration_text = duration_var.get().strip()

    start = float(start_text) if start_text else 0.0
    duration = float(duration_text) if duration_text else BAG_DURATION

    if start < 0:
        raise ValueError("start must be >= 0")
    if duration < 0:
        raise ValueError("duration must be >= 0")

    return start, duration


def reload_plot(start, duration):
    global current_start, current_duration

    root.config(cursor="watch")
    status_var.set("updating...")
    root.update_idletasks()

    try:
        current_start = start
        current_duration = duration
        update_data_cache(start, duration)
        clear_navigation_highlights()
        clear_vertical_lines()
        plot_data()
        highlight_navigation_time(ax2, data, getIndex(ACTIVITY_LOG_TOPIC), current_start, color=options.background_color, alpha=options.background_alpha)
        toggle_vertical_lines(var30)
        status_var.set(f"start={current_start:.2f}s duration={current_duration:.2f}s (cached)")
    except Exception as e:
        status_var.set(f"update failed: {e}")
    finally:
        root.config(cursor="")


def apply_range(event=None):
    try:
        start, duration = parse_range_values()
    except ValueError as e:
        status_var.set(f"invalid range: {e}")
        return

    reload_plot(start, duration)


def show_full_range():
    start_var.set("0")
    duration_var.set(f"{BAG_DURATION:g}")
    apply_range()


def close_all_windows():
    try:
        control_window.destroy()
    except tk.TclError:
        pass
    root.destroy()


root.protocol("WM_DELETE_WINDOW", close_all_windows)

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
var12 = tk.BooleanVar(value=False)
var13 = tk.BooleanVar(value=False)
var14 = tk.BooleanVar(value=False)
var15 = tk.BooleanVar(value=False)
var16 = tk.BooleanVar(value=False)
var17 = tk.BooleanVar(value=False)
var18 = tk.BooleanVar(value=False)
var19 = tk.BooleanVar(value=False)
var20 = tk.BooleanVar(value=False)
var30 = tk.BooleanVar(value=True)

tk.Label(range_frame, text=f"Bag duration: {BAG_DURATION:.2f}s").pack(side=tk.TOP, anchor='w')
tk.Label(range_frame, text="start [s]").pack(side=tk.TOP, anchor='w')
start_entry = tk.Entry(range_frame, textvariable=start_var)
start_entry.pack(side=tk.TOP, fill=tk.X)
tk.Label(range_frame, text="duration [s]").pack(side=tk.TOP, anchor='w')
duration_entry = tk.Entry(range_frame, textvariable=duration_var)
duration_entry.pack(side=tk.TOP, fill=tk.X)
tk.Button(range_frame, text="Redraw", command=apply_range).pack(side=tk.TOP, fill=tk.X, pady=(4, 0))
tk.Button(range_frame, text="Full Range", command=show_full_range).pack(side=tk.TOP, fill=tk.X, pady=(4, 0))
tk.Label(range_frame, textvariable=status_var, justify=tk.LEFT, wraplength=180).pack(side=tk.TOP, anchor='w', pady=(4, 0))

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
checkbox12 = tk.Checkbutton(speed_frame, text="Show /cabot/social_distance_speed", variable=var12, command=lambda: toggle_line(line12, var12, ax1))
checkbox13 = tk.Checkbutton(speed_frame, text="Show /cabot/pure_velocity_obstacle_speed", variable=var13, command=lambda: toggle_line(line13, var13, ax1))
checkbox14 = tk.Checkbutton(speed_frame, text="Show /cabot/combined_speed", variable=var14, command=lambda: toggle_line(line14, var14, ax1))
checkbox15 = tk.Checkbutton(touch_frame, text="Show /cabot/capacitive/touch", variable=var15, command=lambda: toggle_line(line15, var15, ax1))
checkbox16 = tk.Checkbutton(touch_frame, text="Show /cabot/capacitive/touch_raw", variable=var16, command=lambda: toggle_line(line16, var16, ax2))
checkbox17 = tk.Checkbutton(touch_frame, text="Show /cabot/tof/touch", variable=var17, command=lambda: toggle_line(line17, var17, ax1))
checkbox18 = tk.Checkbutton(touch_frame, text="Show /cabot/tof/touch_raw", variable=var18, command=lambda: toggle_line(line18, var18, ax3))
checkbox19 = tk.Checkbutton(speed_frame, text="Show /cabot/low_lidar_speed", variable=var19, command=lambda: toggle_line(line19, var19, ax1))
checkbox20 = tk.Checkbutton(speed_frame, text="Show /cabot/wheelie_speed", variable=var20, command=lambda: toggle_line(line20, var20, ax1))
checkbox30 = tk.Checkbutton(frame, text=f"Show /current_floor", variable=var30, command=lambda: toggle_vertical_lines(var30))

# Create category checkboxes
cmd_vel_var = tk.BooleanVar(value=True)
speed_var = tk.BooleanVar(value=True)
touch_var = tk.BooleanVar(value=True)

cmd_vel_checkbox = tk.Checkbutton(cmd_vel_frame, text="all", variable=cmd_vel_var, command=lambda: toggle_category(cmd_vel_var, [(var1, line1), (var9, line9), (var10, line10), (var11, line11)]))
speed_checkbox = tk.Checkbutton(speed_frame, text="all", variable=speed_var, command=lambda: toggle_category(speed_var, [(var3, line3), (var4, line4), (var6, line6), (var7, line7), (var8, line8), (var12, line12), (var13, line13), (var14, line14), (var19, line19), (var20, line20)]))
touch_checkbox = tk.Checkbutton(touch_frame, text="all", variable=touch_var, command=lambda: toggle_category(touch_var, [(var2, line2), (var5, line5), (var15, line15), (var16, line16), (var17, line17), (var18, line18)]))

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
checkbox12.pack(side=tk.TOP, anchor='w')
checkbox13.pack(side=tk.TOP, anchor='w')
checkbox14.pack(side=tk.TOP, anchor='w')
checkbox19.pack(side=tk.TOP, anchor='w')
checkbox20.pack(side=tk.TOP, anchor='w')
touch_checkbox.pack(side=tk.TOP, anchor='w')
checkbox2.pack(side=tk.TOP, anchor='w')
checkbox5.pack(side=tk.TOP, anchor='w')
checkbox15.pack(side=tk.TOP, anchor='w')
checkbox16.pack(side=tk.TOP, anchor='w')
checkbox17.pack(side=tk.TOP, anchor='w')
checkbox18.pack(side=tk.TOP, anchor='w')

checkbox30.pack(side=tk.TOP, anchor='w')

# Plot data
update_data_cache(current_start, current_duration)
plot_data()

highlight_navigation_time(ax2, data, getIndex(ACTIVITY_LOG_TOPIC), current_start, color=options.background_color, alpha=options.background_alpha)

# Ensure vertical lines are displayed based on the initial checkbox state
toggle_vertical_lines(var30)

start_entry.bind("<Return>", apply_range)
duration_entry.bind("<Return>", apply_range)
status_var.set(f"start={current_start:.2f}s duration={current_duration:.2f}s (cached)")

# Start the Tkinter main loop
root.mainloop()
