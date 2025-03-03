#!/usr/bin/env python3

# Copyright (c) 2020, 2023  Carnegie Mellon University
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


import rclpy
from rclpy.qos import QoSProfile
import yaml
from optparse import OptionParser
import math
import os
import sys
import rclpy.time
from matplotlib import pyplot as plt
import functools

from cabot_common.rosbag2 import BagReader
from tf_bag import BagTfTransformer
from rosidl_runtime_py import message_to_csv
from rosidl_runtime_py import message_to_yaml

from datetime import datetime, timedelta
import pytz

import logging
logging.basicConfig(level=logging.INFO)

parser = OptionParser(usage="""
plot battery states of kx models
Example
{0} -f <bag file>                       # plot battery states (/cabot/battery_states)
{0} -f <bag file> -S                    # plot battery state (/cabot/battery_state)
{0} -f <bag file> -F                    # plot battery states (with fixing invalid values due to bugs)
""".format(sys.argv[0]))

parser.add_option('-f', '--file', type=str, help='bag file to print')
parser.add_option('-s', '--start', type=float, help='start time from the begining', default=0.0)
parser.add_option('-d', '--duration', type=float, help='duration from the start time', default=99999999999999)
parser.add_option('-F', '--fix', action='store_true', help='fix invalid values')
parser.add_option('-T', '--timezone', type=int, help='set timezone default=0', default=0)
parser.add_option('-S', '--single', action='store_true', help='use /cabot/battery_state instead of /cabot/battery_states')

(options, args) = parser.parse_args()

if not options.file:
    parser.print_help()
    sys.exit(0)

logging.info(options)
bagfilename = options.file
reader = BagReader(bagfilename)

if options.single:
    topics = ["/cabot/battery_state"]
else:
    topics = ["/cabot/battery_states"]

reader.set_filter_by_topics(topics)
reader.set_filter_by_options(options)  # filter by start and duration

NUM_OF_DATA = 100
ts = tuple([[] for i in range(NUM_OF_DATA)])
ds = tuple([[] for i in range(NUM_OF_DATA)])
voltages = [[] for _ in range(NUM_OF_DATA)]
temperatures = [[] for _ in range(NUM_OF_DATA)]
currents = [[] for _ in range(NUM_OF_DATA)]
percentages = [[] for _ in range(NUM_OF_DATA)]
serial_numbers = []

while reader.has_next():
    try:
        (topic, msg, t, st) = reader.serialize_next()
    except:
        continue
    if not topic:
        continue
    dt_object_utc = datetime.utcfromtimestamp(t).replace(tzinfo=pytz.utc)
    dt_object_jst = dt_object_utc + timedelta(hours=options.timezone)
    
    if options.single:
        battery = msg
        voltages[0].append(battery.voltage)
        if not options.fix or (battery.temperature < 100):
            temperatures[0].append(battery.temperature)
        else:
            logging.error(f"battery temperature is too high: {battery.temperature}")

        if not options.fix or battery.current < 32.7685:
            currents[0].append(battery.current)
        else:
            currents[0].append(battery.current - 65.536)
            logging.error(f"battery current is too high: {battery.current} -> converted to {-battery.current + 65.536}")

        if not options.fix or battery.percentage < 1.01:
            percentages[0].append(battery.percentage)
        else:
            logging.error(f"battery percentage is too high: {battery.percentage}")

        if len(serial_numbers) == 0:
            serial_numbers.append(format(int(battery.serial_number), '04x'))
    else:
        for i, battery in enumerate(msg.batteryarray):
            voltages[i].append(battery.voltage)
            if not options.fix or (battery.temperature < 100):
                temperatures[i].append(battery.temperature)
            else:
                logging.error(f"battery[{i}] temperature is too high: {battery.temperature}")

            if not options.fix or battery.current < 32.7685:
                currents[i].append(battery.current)
            else:
                currents[i].append(battery.current - 65.536)
                logging.error(f"battery[{i}] current is too high: {battery.current} -> converted to {-battery.current + 65.536}")

            if not options.fix or battery.percentage < 1.01:
                percentages[i].append(battery.percentage)
            else:
                logging.error(f"battery[{i}] percentage is too high: {battery.percentage}")

            if len(serial_numbers) <= i:
                serial_numbers.append(format(int(battery.serial_number), '04x'))

# Plotting
fig, axs = plt.subplots(4, 1, figsize=(10, 8), sharex=True)

for i in range(len(serial_numbers)):
    axs[0].plot(voltages[i], label=f'Voltage {serial_numbers[i]}')
    axs[1].plot(temperatures[i], label=f'Temperature {serial_numbers[i]}')
    axs[2].plot(currents[i], label=f'Current {serial_numbers[i]}')
    axs[3].plot(percentages[i], label=f'Percentage {serial_numbers[i]}')

axs[0].set_ylabel('Voltage (V)')
axs[0].legend()

axs[1].set_ylabel('Temperature (°C)')
axs[1].legend()

axs[2].set_ylabel('Current (A)')
axs[2].legend()

axs[3].set_ylabel('Percentage (%)')
axs[3].legend()

plt.xlabel('Time')
plt.xticks(rotation=45)
plt.tight_layout()
plt.show()

"""
# /cabot/battery_states format

batteryarray:
- header:
    stamp:
      sec: 1739362811
      nanosec: 15611351
    frame_id: ''
  voltage: 28.847000122070312
  temperature: 27.700000762939453
  current: 3.002000093460083
  charge: 0.0
  capacity: 0.0
  design_capacity: 0.0
  percentage: 1.0
  power_supply_status: 0
  power_supply_health: 0
  power_supply_technology: 0
  present: false
  cell_voltage: []
  cell_temperature: []
  location: '1'
  serial_number: '3013'
- header:
    stamp:
      sec: 1739362811
      nanosec: 20029952
    frame_id: ''
  voltage: 29.062000274658203
  temperature: 31.799999237060547
  current: 0.0
  charge: 0.0
  capacity: 0.0
  design_capacity: 0.0
  percentage: 0.9900000095367432
  power_supply_status: 0
  power_supply_health: 0
  power_supply_technology: 0
  present: false
  cell_voltage: []
  cell_temperature: []
  location: '2'
  serial_number: '3331'
- header:
    stamp:
      sec: 1739362811
      nanosec: 20316629
    frame_id: ''
  voltage: 28.964000701904297
  temperature: 26.600000381469727
  current: 0.0
  charge: 0.0
  capacity: 0.0
  design_capacity: 0.0
  percentage: 0.9900000095367432
  power_supply_status: 0
  power_supply_health: 0
  power_supply_technology: 0
  present: false
  cell_voltage: []
  cell_temperature: []
  location: '3'
  serial_number: '3738'
- header:
    stamp:
      sec: 1739362811
      nanosec: 21118610
    frame_id: ''
  voltage: 29.05900001525879
  temperature: 26.600000381469727
  current: 0.0
  charge: 0.0
  capacity: 0.0
  design_capacity: 0.0
  percentage: 0.9900000095367432
  power_supply_status: 0
  power_supply_health: 0
  power_supply_technology: 0
  present: false
  cell_voltage: []
  cell_temperature: []
  location: '4'
  serial_number: '3637'
"""