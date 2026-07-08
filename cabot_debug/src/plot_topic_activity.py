#!/usr/bin/env python3

###############################################################################
# Copyright (c) 2026  Carnegie Mellon University and Miraikan
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

import os
import sys
from optparse import OptionParser

from matplotlib import pyplot as plt

from cabot_common.rosbag2 import BagReader


DEFAULT_TOPICS = [
    "/cabot/cmd_vel",
    "/cmd_vel",
    "/cabot/lidar_speed",
    "/cabot/people_speed",
    "/cabot/tf_speed",
    "/cabot/low_lidar_speed",
    "/cabot/odometry/filtered",
    "/cabot/odom_raw",
    "/cabot/imu/data",
    "/velodyne_points",
    "/wireless/beacons",
    "/cabot/capacitive/touch",
    "/cabot/tof/touch",
]

TF_TOPIC_NAMES = {
    "/tf",
    "/tf_static",
    "/local/tf",
    "/local/tf_static",
    "/tf_temp",
    "/tf_static_temp",
    "tf",
    "tf_static",
    "local/tf",
    "local/tf_static",
    "tf_temp",
    "tf_static_temp",
}


parser = OptionParser(usage="""
Plot topic activity as a raster chart.
Example
{0} -f <bag file>
{0} -f <bag file> -t /cabot/touch -t /cabot/activity_log
{0} -f <bag file> -t "odom -> base_footprint" -t "map -> odom"
{0} -f <bag file> --no-default-topics -t /cmd_vel -t /odom
{0} -f <bag file> -s 30 -d 20 -O activity.png --no-show
""".format(sys.argv[0]))

parser.add_option('-f', '--file', type=str, help='bag file to be processed')
parser.add_option('-s', '--start', type=float, help='start time from the beginning', default=0.0)
parser.add_option('-d', '--duration', type=float, help='duration from the start time', default=99999999999999)
parser.add_option('-t', '--topic', action='append', default=[], help='additional topic to plot')
parser.add_option('--no-default-topics', action='store_true', help='plot only topics passed by --topic')
parser.add_option('--target-columns', type=int, default=4000, help='target horizontal resolution for activity compression')
parser.add_option('-O', '--output', type=str, help='save the figure to a file')
parser.add_option('--no-show', action='store_true', help='do not open the plot window')

(options, args) = parser.parse_args()

if not options.file:
    parser.print_help()
    sys.exit(0)


def unique_topics(topics):
    result = []
    seen = set()
    for topic in topics:
        if topic in seen:
            continue
        seen.add(topic)
        result.append(topic)
    return result


def normalize_tf_pair(item):
    if "->" not in item:
        return None
    parent, child = item.split("->", 1)
    parent = parent.strip()
    child = child.strip()
    if not parent or not child:
        return None
    return f"{parent} -> {child}"


def is_tf_topic(topic_name):
    return topic_name in TF_TOPIC_NAMES


def tf_pair_label(transform):
    return f"{transform.header.frame_id} -> {transform.child_frame_id}"


def build_requested_topics():
    topics = []
    if not options.no_default_topics:
        topics.extend(DEFAULT_TOPICS)
    topics.extend(options.topic)
    return unique_topics(topics)


def split_requested_items(items):
    normal_topics = []
    tf_pairs = []
    for item in items:
        tf_pair = normalize_tf_pair(item)
        if tf_pair is None:
            normal_topics.append(item)
        else:
            tf_pairs.append(tf_pair)
    return normal_topics, tf_pairs


def get_plot_window(reader):
    bag_duration = reader.bag_duration()
    window_start = max(0.0, options.start)
    window_end = min(float(window_start + options.duration), float(bag_duration))
    if window_end <= window_start:
        window_end = window_start + 1.0
    return window_start, window_end


def get_activity_resolution(window):
    duration = max(window[1] - window[0], 1e-6)
    target_columns = max(1, options.target_columns)
    return duration / target_columns


def collect_activity():
    reader = BagReader(options.file)
    available_topics = {info.name for info in reader.topic_types}
    requested_items = build_requested_topics()
    requested_topics, requested_tf_pairs = split_requested_items(requested_items)
    plot_topics = [topic for topic in requested_topics if topic in available_topics]
    missing_topics = [topic for topic in requested_topics if topic not in available_topics]
    tf_topics = [topic for topic in sorted(available_topics) if is_tf_topic(topic)]

    if missing_topics:
        print("warning: topics not found in bag:", file=sys.stderr)
        for topic in missing_topics:
            print(f"  {topic}", file=sys.stderr)

    if requested_tf_pairs and not tf_topics:
        print("warning: no TF topics found in bag for requested frame pairs", file=sys.stderr)

    plot_items = []
    for item in requested_items:
        tf_pair = normalize_tf_pair(item)
        if tf_pair is not None:
            plot_items.append(tf_pair)
        elif item in plot_topics:
            plot_items.append(item)

    if not plot_items:
        print("error: no matching topics found in bag", file=sys.stderr)
        sys.exit(1)

    activity = {item: [] for item in plot_items}
    requested_tf_pair_set = set(requested_tf_pairs)
    filter_topics = unique_topics(plot_topics + tf_topics)

    if filter_topics:
        reader.set_filter_by_topics(filter_topics)
        reader.set_filter_by_options(options)

        while reader.has_next():
            try:
                topic, msg, t, st = reader.serialize_next()
            except Exception:
                continue
            if not topic:
                continue
            if topic in activity:
                activity[topic].append(st)
            if topic in tf_topics:
                for transform in msg.transforms:
                    pair_label = tf_pair_label(transform)
                    if pair_label in requested_tf_pair_set:
                        activity[pair_label].append(st)

    return plot_items, activity, get_plot_window(reader)


def build_activity_segments(times, window, resolution):
    if not times:
        return []

    start_limit, end_limit = window
    min_width = max(resolution, 1e-6)
    segments = []
    segment_start = None
    segment_end = None

    for t in times:
        if t < start_limit or t > end_limit:
            continue

        interval_start = max(t, start_limit)
        interval_end = min(t + min_width, end_limit)

        if segment_start is None:
            segment_start = interval_start
            segment_end = interval_end
            continue

        if interval_start <= segment_end:
            if interval_end > segment_end:
                segment_end = interval_end
            continue

        segments.append((segment_start, max(segment_end - segment_start, min_width)))
        segment_start = interval_start
        segment_end = interval_end

    if segment_start is not None:
        segments.append((segment_start, max(segment_end - segment_start, min_width)))

    return segments


def plot_activity(topics, activity, window):
    figure_height = max(4.0, 0.6 * len(topics) + 1.5)
    fig, ax = plt.subplots(figsize=(20, figure_height))

    positions = list(range(len(topics)))
    colors = [f"C{index % 10}" for index in positions]
    labels = [f"{topic} ({len(activity[topic])})" for topic in topics]
    resolution = get_activity_resolution(window)

    for position, topic, color in zip(positions, topics, colors):
        segments = build_activity_segments(activity[topic], window, resolution)
        if segments:
            ax.broken_barh(segments, (position - 0.4, 0.8), facecolors=color, edgecolors=color, linewidth=0.2)

    ax.set_xlim(window[0], window[1])
    ax.set_yticks(positions)
    ax.set_yticklabels(labels)
    ax.invert_yaxis()
    ax.grid(axis="x", linestyle=":", alpha=0.5)
    ax.set_xlabel("time from bag start [s]")
    ax.set_ylabel("topic or tf pair (message count)")
    ax.set_title(f"Topic activity: {os.path.basename(options.file)}")
    fig.tight_layout()
    return fig


topics, activity, window = collect_activity()
figure = plot_activity(topics, activity, window)

if options.output:
    figure.savefig(options.output, dpi=150, bbox_inches="tight")

if not options.no_show:
    plt.show()
