#!/usr/bin/env python3

import argparse
import math
import matplotlib.pyplot as plt
import sys
import struct
from datetime import datetime
from matplotlib import gridspec  # Add this import

plt.rcParams.update({
    "font.size": 8,         # overall font size
    "axes.titlesize": 10,   # title
    "axes.labelsize": 9,    # x/y labels
    "xtick.labelsize": 7,   # x-tick labels
    "ytick.labelsize": 7,   # y-tick labels
    "legend.fontsize": 7,   # legend text
    "figure.titlesize": 11  # figure title
})


def parse_line(line):
    """Parse a line in the format '(timestamp) bus_id message_id#data'."""
    try:
        timestamp, rest = line.split(')', 1)
        timestamp = timestamp.strip('(').strip()
        bus_id, message = rest.strip().split(' ', 1)
        message_id, data = message.split('#', 1)
        return timestamp, bus_id, message_id, data
    except ValueError:
        return None  # Return None if the line is not in the expected format

def show():
    """Finalize and display the plot."""
    print(f"Setting legend")
    plt.tight_layout()
    print(f"Showing plot")
    plt.show()


def main():
    parser = argparse.ArgumentParser(description="Plot battery states.")
    parser.add_argument('-f', '--file', nargs="+", help="Paths to the input files (can be specified multiple times)")
    parser.add_argument('-s', '--start-time', type=float, default=0.0, help="Start time for plotting (seconds from min_time)")
    parser.add_argument('-d', '--duration', type=float, default=None, help="Duration for plotting (seconds)")
    args = parser.parse_args()

    touches = []  # List to store touch states
    handle_states = []  # List to store handle states
    all_timestamps = []  # List to store all timestamps

    total_lines = 0
    parsed_lines = 0

    if args.file:
        files = sorted(args.file)
        print(f"Processing files: {files}")
        for file_path in files:
            # Count total lines
            with open(file_path, 'r') as file:
                total_lines += sum(1 for _ in file)
        print(f"Total lines to process: {total_lines}")

        # Process files and display progress
        progress_threshold = max(1, total_lines // 20)  # 5% of total lines
        count = 0
        line_count = 0

        plots = []
        for file_path in files:
            with open(file_path, 'r') as file:
                for i, line in enumerate(file, start=1):
                    line_count += 1
                    if line_count % progress_threshold == 0 or line_count == total_lines:
                        print(f"Progress: {line_count}/{total_lines} lines ({(line_count / total_lines) * 100:.2f}%)")
                    parsed = parse_line(line.strip())
                    if parsed:
                        parsed_lines += 1
                        timestamp, bus_id, message_id, data = parsed
                        all_timestamps.append(timestamp)
                        if message_id == '080':
                            tof_raw, cap_raw, cap = process_touch_states(timestamp, data)  # Define this function or replace it with the correct one
                            touches.append((timestamp, cap_raw, cap))
                        if message_id == '481':
                            s1, s2, s3 = process_handle_states(timestamp, data)  # Define this function or replace it with the correct one
                            handle_states.append((timestamp, s1, s2, s3))
                        else:
                            pass  # print(f"Unhandled message ID: {message_id}")
                    else:
                        print(f"Invalid line format: {line.strip()}")

        min_time = min(float(ts) for ts in all_timestamps)
        start_time = args.start_time
        end_time = start_time + args.duration if args.duration else float('inf')

        # Filter touches
        touches = [(float(t[0]) - min_time, t[1], t[2]) for t in touches if start_time <= float(t[0]) - min_time <= end_time]

        # Filter handle states
        handle_states = [
            (float(h[0]) - min_time, h[1], h[2], h[3])
            for h in handle_states if start_time <= float(h[0]) - min_time <= end_time
        ]

        # Create a figure with 4 vertical subplots
        print(f"Plotting touch and handle states {min_time=}")
        fig, axs = plt.subplots(9, 1, figsize=(10, 15), sharex=True)
        fig.subplots_adjust(hspace=0.4)  # Adjust space between subplots

        # Plot touch states
        i = 0
        if touches:
            timestamps = [touch[0] for touch in touches]
            touch_raw_states = [touch[1] for touch in touches]
            axs[i].plot(timestamps, touch_raw_states, label="Touch Raw States", color='purple')
            axs[i].set_ylabel("Touch Raw State")
            axs[i].set_title("Touch Raw States over Time")
            axs[i].grid(True)
            axs[i].set_ylim(-140, 140)
            axs[i].legend()

            i += 1
            touch_states = [touch[2] for touch in touches]
            axs[i].plot(timestamps, touch_states, label="Touch States", color='blue')
            axs[i].set_ylabel("Touch State")
            axs[i].set_title("Touch States over Time")
            axs[i].grid(True)
            axs[i].set_ylim(-0.1, 1.1)
            axs[i].legend()

        # Plot handle state S1
        if handle_states:
            timestamps = [state[0] for state in handle_states]

            label = ["MTP", "MULT", "PWR", "ACAL_FAIL", "BC_OUT"]
            for j in range(0, 5):
                i += 1
                s1_states = [state[1][j] for state in handle_states]
                axs[i].plot(timestamps, s1_states, label=label[j])
                axs[i].set_ylabel(f"Handle State {label[j]}")
                axs[i].set_title(f"Handle State {label[j]} over Time")
                axs[i].grid(True)
                axs[i].set_ylim(-0.1, 1.1)
                axs[i].legend()

            # Plot handle state S2
            s2_states = [state[2] for state in handle_states]
            i += 1
            axs[i].plot(timestamps, s2_states, label="Noise", color='green')
            axs[i].set_ylabel("Noise")
            axs[i].set_title("Noise over Time")
            axs[i].grid(True)
            axs[i].set_ylim(-0.1, 1.1)
            axs[i].legend()

            # Plot handle state S3
            s3_states = [state[3] for state in handle_states]
            i += 1
            axs[i].plot(timestamps, s3_states, label="Calibration State", color='blue')
            axs[i].set_xlabel("Time")
            axs[i].set_ylabel("Calibration State")
            axs[i].set_title("Calibration State over Time")
            axs[i].grid(True)
            axs[i].set_ylim(-0.1, 1.1)
            axs[i].legend()

        # Show the plots
        plt.show()


def process_touch_states(timestamp, data):
    """Process touch states from the CAN frame."""
    tof_raw = int(data[0:4], 16)
    cap_raw = struct.unpack('b', bytes.fromhex(data[4:6]))[0]
    cap = int(data[6:8], 16)
    # print(f"Processing touch states at {timestamp}: {status4:02x}")
    return tof_raw, cap_raw, cap

def process_handle_states(timestamp, data):
    """Process handle states from the CAN frame."""
    status1 = int(data[0:2], 16)

    MTP = (status1 >> 1) & 1
    MULT = (status1 >> 2) & 1
    PWR = (status1 >> 4) & 1
    ACAL_FAIL = (status1 >> 5) & 1
    BC_OUT = (status1 >> 6) & 1

    status2 = int(data[2:4], 16) & 1
    status3 = int(data[4:6], 16) & 1
    return [MTP, MULT, PWR, ACAL_FAIL, BC_OUT], status2, status3

if __name__ == "__main__":
    main()

