#!/usr/bin/env python3

###############################################################################
# Copyright (c) 2026  Carnegie Mellon University
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

import argparse
import math
import re
from collections import Counter
from datetime import datetime
from pathlib import Path

from matplotlib import pyplot as plt
from matplotlib.ticker import FuncFormatter


LINE_RE = re.compile(
    r"^\((?P<timestamp>\d+(?:\.\d+)?)\)\s+"
    r"(?P<interface>can[01])\s+"
    r"(?P<frame>[0-9A-Fa-f]+#[0-9A-Fa-f]*)$"
)


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "Count candump records per second for can0 and can1, "
            "with ros_time-style range filtering."
        )
    )
    parser.add_argument(
        "logfile",
        nargs="?",
        type=Path,
        help="Path to a candump-style log file.",
    )
    parser.add_argument(
        "-f",
        "--file",
        dest="logfile_option",
        type=Path,
        help="Path to a candump-style log file.",
    )
    parser.add_argument(
        "-s",
        "--start",
        type=float,
        default=0.0,
        help="Start time from the beginning of the log in seconds.",
    )
    parser.add_argument(
        "-d",
        "--duration",
        type=float,
        default=99999999999999.0,
        help="Duration from the start time in seconds.",
    )
    parser.add_argument(
        "-o",
        "--output",
        type=Path,
        help="Write the graph to a PNG file instead of showing it interactively.",
    )
    parser.add_argument(
        "--title",
        default=None,
        help="Optional plot title.",
    )
    args = parser.parse_args()
    args.logfile = args.logfile_option or args.logfile
    if args.logfile is None:
        parser.error("logfile is required")
    return args


def load_records(logfile):
    records = []
    first_timestamp = None
    invalid_lines = 0

    with logfile.open("r", encoding="utf-8") as stream:
        for line in stream:
            match = LINE_RE.match(line.strip())
            if match is None:
                invalid_lines += 1
                continue

            timestamp = float(match.group("timestamp"))
            interface = match.group("interface")
            records.append((timestamp, interface))

            if first_timestamp is None:
                first_timestamp = timestamp

    if first_timestamp is None:
        raise ValueError(f"No valid can0/can1 records found in {logfile}")

    if invalid_lines:
        print(f"warning: ignored {invalid_lines} invalid line(s)")

    return records, first_timestamp


def build_series(records, first_timestamp, start, duration):
    start_epoch = first_timestamp + start
    end_epoch = first_timestamp + start + duration
    counts = {
        "can0": Counter(),
        "can1": Counter(),
    }
    min_second = None
    max_second = None

    for timestamp, interface in records:
        if timestamp < start_epoch:
            continue
        if timestamp >= end_epoch:
            continue

        second = math.floor(timestamp)
        counts[interface][second] += 1

        if min_second is None or second < min_second:
            min_second = second
        if max_second is None or second > max_second:
            max_second = second

    if min_second is None or max_second is None:
        raise ValueError("No data in the requested range")

    start_second = min_second
    end_second = max_second
    seconds = list(range(start_second, end_second + 1))
    ros_times = [float(second) for second in seconds]
    wall_times = [datetime.fromtimestamp(second) for second in seconds]
    can0 = [counts["can0"].get(second, 0) for second in seconds]
    can1 = [counts["can1"].get(second, 0) for second in seconds]
    return ros_times, wall_times, can0, can1


def plot_counts(ros_times, wall_times, can0, can1, title):
    fig, ax = plt.subplots(figsize=(16, 6))

    ax.step(ros_times, can0, where="post", label="can0", linewidth=1.8)
    ax.step(ros_times, can1, where="post", label="can1", linewidth=1.8)

    ax.set_xlabel("ros_time [s]")
    ax.set_ylabel("Messages per second")
    ax.set_title(title)
    ax.grid(True, linestyle="--", alpha=0.4)
    ax.legend()
    ax.xaxis.set_major_formatter(FuncFormatter(lambda value, _: f"{value:.0f}"))

    top_ax = ax.twiny()
    top_ax.set_xlim(ax.get_xlim())

    if ros_times:
        tick_count = min(10, len(ros_times))
        if tick_count == 1:
            indices = [0]
        else:
            step = (len(ros_times) - 1) / (tick_count - 1)
            indices = []
            for i in range(tick_count):
                index = round(i * step)
                if not indices or index != indices[-1]:
                    indices.append(index)
        tick_positions = [ros_times[index] for index in indices]
        tick_labels = [wall_times[index].strftime("%H:%M:%S") for index in indices]
        top_ax.set_xticks(tick_positions)
        top_ax.set_xticklabels(tick_labels, rotation=30, ha="left")

    top_ax.set_xlabel("Wall time")
    fig.tight_layout()
    return fig


def main():
    args = parse_args()
    records, first_timestamp = load_records(args.logfile)
    ros_times, wall_times, can0, can1 = build_series(
        records,
        first_timestamp,
        args.start,
        args.duration,
    )

    title = args.title
    if title is None:
        title = f"CAN message count per second: {args.logfile.name}"

    fig = plot_counts(ros_times, wall_times, can0, can1, title)

    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(args.output, dpi=150)
        print(f"saved {args.output}")
        return

    plt.show()


if __name__ == "__main__":
    main()
