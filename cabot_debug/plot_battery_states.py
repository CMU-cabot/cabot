#!/usr/bin/env python3

import argparse
import math
import matplotlib.pyplot as plt
import sys
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

def parse_battery_status(data, fix_data):
    """Parse battery status data from the CAN frame."""
    if len(data) != 16:
        return None  # Ensure data length is valid
    try:
        # Adjust parsing to handle higher-order bits coming later
        voltage = int(data[2:4] + data[0:2], 16) / 1000.0  # Convert mV to V
        b = bytes.fromhex(data[6:8] + data[4:6])
        current = int.from_bytes(b, byteorder='big', signed=True) / 1000.0
        # current_raw = int(data[6:8] + data[4:6], 16)
        # current = (current_raw - 0x10000) / 1000.0 if current_raw > 0x7FFF else current_raw / 1000.0  # Signed int16
        percentage = int(data[10:12] + data[8:10], 16)  # Convert to percentage
        temperature = int(data[14:16] + data[12:14], 16) / 10.0 - 273.1  # Convert to Celsius

        # Discard invalid values
        if fix_data:
            if temperature > 100.0:
                temperature = float('nan')
            if percentage > 100.0:
                percentage = float('nan')
            if abs(current) > 20.0:  # Discard current values greater than 20A
                current = float('nan')

        return voltage, current, percentage, temperature
    except ValueError:
        return None  # Return None if parsing fails

def process_battery_data(timestamp, message_id, data, records, temp_statuses, fix_data):
    """Process battery data based on the message ID and temporarily store it."""
    battery_status = parse_battery_status(data, fix_data)
    if battery_status:
        voltage, current, percentage, temperature = battery_status
        # print(f"Battery ID: {message_id}, Voltage: {voltage:.2f}V, Current: {current:.2f}A, "
        #       f"Percentage: {percentage:.2f}%, Temperature: {temperature:.2f}°C")
        temp_statuses[message_id] = {
            "timestamp": timestamp,
            "message_id": message_id,
            "voltage": voltage,
            "current": current,
            "percentage": percentage,
            "temperature": temperature
        }
    else:
        print(f"Invalid battery status data for ID {message_id}: {data}")

def process_battery_serial_numbers(data, serial_numbers, temp_statuses, records):
    """Process battery serial numbers from message ID 520 and add complete records."""
    if len(data) != 16:
        print(f"Invalid serial number data: {data}")
        return
    try:
        serial_numbers['518'] = data[2:4] + data[0:2]
        serial_numbers['519'] = data[6:8] + data[4:6]
        serial_numbers['51A'] = data[10:12] + data[8:10]
        serial_numbers['51B'] = data[14:16] + data[12:14]

        # Add complete records to the list
        for battery_id in ['518', '519', '51A', '51B']:
            if battery_id in temp_statuses:
                record = temp_statuses[battery_id].copy()
                record["serial_number"] = serial_numbers[battery_id]
                records.append(record)
    except ValueError:
        print(f"Error parsing serial number data: {data}")

def sample_can_message_timestamps(all_timestamps):
    """Sample timestamps of other CAN messages every second."""
    sampled_seconds = set()
    for ts in all_timestamps:
        second = int(float(ts))  # Convert to integer seconds
        sampled_seconds.add(second)
    return sampled_seconds

def prepare(total):
    """Prepare the figure with gridspec for multiple plots."""
    global fig, gs
    fig = plt.figure(figsize=(6 * total, 12))
    gs = gridspec.GridSpec(4, total, figure=fig)

def plot_battery_status(records, serial_numbers, all_timestamps, count, total):
    """Plot battery status for the specified serial number using gridspec."""

    # Define a fixed color mapping for serial numbers
    color_mapping = {
        serial_number: plt.cm.tab10(i % 10) for i, serial_number in enumerate(serial_numbers)
    }

    ax1 = fig.add_subplot(gs[0, count])
    ax2 = fig.add_subplot(gs[1, count])
    ax3 = fig.add_subplot(gs[2, count])
    ax4 = fig.add_subplot(gs[3, count])

    flag = False

    print(f"Plotting data for serial numbers: {serial_numbers}")
    for serial_number in serial_numbers:
        filtered_records = [record for record in records if record.get("serial_number") == serial_number]
        print(f"Plotting data for serial number: {serial_number} with {len(filtered_records)} records.")
        if not filtered_records:
            print(f"No records found for serial number: {serial_number}")
            continue

        flag = True
        timestamps = [float(record["timestamp"]) for record in filtered_records]
        voltages = [record["voltage"] for record in filtered_records]
        currents = [record["current"] for record in filtered_records]
        percentages = [record["percentage"] for record in filtered_records]
        temperatures = [record["temperature"] for record in filtered_records]

        # Sample CAN message timestamps
        sampled_seconds = sample_can_message_timestamps(all_timestamps)

        # Determine xlim based on sampled_seconds
        if sampled_seconds:
            min_time = min(sampled_seconds)
            max_time = max(sampled_seconds)
        else:
            min_time = min(timestamps)
            max_time = max(timestamps)

        print(f"Plotting voltages")
        # Voltage plot
        ax1.plot(timestamps, voltages, label=f"Voltage (V) {serial_number}", color=color_mapping[serial_number])
        ax1.set_ylabel("Voltage (V)")
        ax1.set_title("Voltage over Time")
        ax1.grid(True)
        ax1.set_xticks([])  # Remove X-axis labels
        ax1.set_xlim(min_time, max_time)
        ax1.set_ylim([23, 30])
        ax1.legend()

        print(f"Plotting currents")
        # Current plot
        ax2.plot(timestamps, currents, label=f"Current (A) {serial_number}", color=color_mapping[serial_number])
        ax2.set_ylabel("Current (A)")
        ax2.set_title("Current over Time")
        ax2.grid(True)
        ax2.set_xticks([])  # Remove X-axis labels
        ax2.set_xlim(min_time, max_time)
        ax2.set_ylim([-5, 5])
        ax2.legend()

        print(f"Plotting percentages")
        # Percentage plot
        ax3.plot(timestamps, percentages, label=f"Percentage (%) {serial_number}", color=color_mapping[serial_number])
        ax3.set_ylabel("Percentage (%)")
        ax3.set_title("Percentage over Time")
        ax3.grid(True)
        ax3.set_xticks([])  # Remove X-axis labels
        ax3.set_xlim(min_time, max_time)
        ax3.set_ylim([0, 100])
        ax3.legend()

        print(f"Plotting temperatures")
        # Temperature plot
        ax4.plot(timestamps, temperatures, label=f"Temperature (°C) {serial_number}", color=color_mapping[serial_number])
        ax4.set_xlabel("Time")
        ax4.set_ylabel("Temperature (°C)")
        ax4.set_title("Temperature over Time")
        ax4.grid(True)
        ax4.set_xlim(min_time, max_time)
        ax4.set_ylim([0, 50])
        ax4.legend()

    if flag:
        print(f"Setting X-axis limits")
        # Format X-axis tick labels
        ax4.set_xticks(
            range(int(min_time), int(max_time) + 1, max(1, (int(max_time) - int(min_time)) // 10))
        )
        ax4.set_xticklabels(
            [datetime.fromtimestamp(ts).strftime('%Y-%m-%d %H:%M:%S') for ts in range(
                int(min_time), int(max_time) + 1, max(1, (int(max_time) - int(min_time)) // 10)
            )],
            rotation=90
        )


def show():
    """Finalize and display the plot."""
    print(f"Setting legend")
    plt.tight_layout()
    print(f"Showing plot")
    plt.show()


def main():
    parser = argparse.ArgumentParser(description="Plot battery states.")
    parser.add_argument('-f', '--file', nargs="+", help="Paths to the input files (can be specified multiple times)")
    parser.add_argument('-s', '--serial', type=str, nargs="+", help="Serial number(s) to filter and plot")
    parser.add_argument('-a', '--all', action="store_true", help="Plot all serial numbers")
    parser.add_argument('-F', '--fix-data', action="store_true", help="Fix invalid data")
    args = parser.parse_args()

    records = []  # List to store battery data with timestamps
    serial_numbers = {}  # Dictionary to store serial numbers for each battery ID
    all_serial_numbers = {}  # Dictionary to store all serial numbers
    temp_statuses = {}  # Temporary storage for the last statuses of batteries
    all_timestamps = []  # List to store all timestamps from CAN messages

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
                        if message_id in ['518', '519', '51A', '51B']:
                            process_battery_data(timestamp, message_id, data, records, temp_statuses, args.fix_data)
                        elif message_id == '520':
                            process_battery_serial_numbers(data, serial_numbers, temp_statuses, records)
                        else:
                            pass  # print(f"Unhandled message ID: {message_id}")
                    else:
                        print(f"Invalid line format: {line.strip()}")
            all_serial_numbers[file_path] = serial_numbers.copy()  # Store serial numbers for each file

            if args.serial and set(args.serial) & set(serial_numbers.values()):
                plots.append([records.copy(), args.serial, all_timestamps.copy(), count])
                count += 1
            elif args.all:
                plots.append([records.copy(), serial_numbers.values(), all_timestamps.copy(), count])
                count += 1

            records.clear()
            all_timestamps = []
            print(f"parsed {file_path} {parsed_lines} lines {len(plots)=}, {count=}")

        if args.serial or args.all:
            prepare(count)
            for d in plots:
                plot_battery_status(d[0], d[1], d[2], d[3], count)
            show()

    # Print serial numbers if no plotting option is specified
    if not args.serial and not args.all:
        print("Battery Serial Numbers:")
        serial_set = set()
        for file_path, serial_numbers in all_serial_numbers.items():
            print(f"File: {file_path}")
            for battery_id, serial in serial_numbers.items():
                print(f"Battery ID: {battery_id}, Serial Number: {serial}")
                serial_set.add(serial)
        print(f"Unique Serial Numbers: {' '.join(list(serial_set))}")


if __name__ == "__main__":
    main()
