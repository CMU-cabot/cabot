#!/usr/bin/env python3

import logging
import os
from optparse import OptionParser
import sys
import rclpy
from tf_transformations import quaternion_multiply, euler_from_quaternion

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from cabot_common.rosbag2 import BagReader

import cabot_ui.geoutil
from cabot_ui.stop_reasoner import StopReasoner, StopReason, StopReasonFilter

logging.basicConfig(format='%(name)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG if 'DEBUG' in os.environ else logging.INFO)

parser = OptionParser(usage="""
print/plot topics
Example
{0} -f <bag file>                       # run stop reasoner
(support only one topic)
""".format(sys.argv[0]))

parser.add_option('-f', '--file', type=str, help='bag file to print')
parser.add_option('-s', '--start', type=float, help='start time from the begining', default=0.0)
parser.add_option('-d', '--duration', type=float, help='duration from the start time', default=99999999999999)

(options, args) = parser.parse_args()

if not options.file:
    parser.print_help()
    sys.exit(0)

reader = BagReader(options.file)

topic_types = reader.get_all_topics_and_types()
type_map = {topic_types[i].name: topic_types[i].type for i in range(len(topic_types))}

#tf_transformer = tf_bag.BagTfTransformer(bag)
tf_transformer = None
reasoner = StopReasoner(tf_transformer)

prev_code = None
prev_duration = 0

ODOM_TOPIC = "/cabot/odom_raw"
EVENT_TOPIC = "/cabot/event"
CMD_VEL_TOPIC = "/cmd_vel"
PEOPLE_SPEED_TOPIC = "/cabot/people_speed"
TF_SPEED_TOPIC = "/cabot/tf_speed"
TOUCH_SPEED_TOPIC = "/cabot/touch_speed_switched"
RECEIVED_GLOBAL_PLAN = "/received_global_plan"
REPLAN_REASON_TOPIC = "/replan_reason"
CURRENT_FRAME_TOPIC = "/current_frame"

all_topics = [
    ODOM_TOPIC,
    EVENT_TOPIC,
    CMD_VEL_TOPIC,
    PEOPLE_SPEED_TOPIC,
    TF_SPEED_TOPIC,
    TOUCH_SPEED_TOPIC,
    RECEIVED_GLOBAL_PLAN,
    REPLAN_REASON_TOPIC,
    CURRENT_FRAME_TOPIC,
]

stop_reason_filter = StopReasonFilter(["NO_NAVIGATION", "NOT_STOPPED", "NO_TOUCH", "STOPPED_BUT_UNDER_THRESHOLD"])
#stop_reason_filter = StopReasonFilter([])

reader.set_filter_by_topics(all_topics)
reader.set_filter_by_options(options)  # filter by start and duration

prev_code = None

rclpy.init()

while reader.has_next() and rclpy.ok():
    try:
        (topic, msg, t, st) = reader.serialize_next()
        if t is None:
            continue
        t = t*1e9
        reasoner.update_time(t)

        code = None
        duration = 0

        if topic == ODOM_TOPIC:
            reasoner.input_odom(msg)
        elif topic == EVENT_TOPIC:
            reasoner.input_event(msg)
        elif topic == RECEIVED_GLOBAL_PLAN:
            reasoner.input_global_plan(msg)
        elif topic == CMD_VEL_TOPIC:
            reasoner.input_cmd_vel(msg)
        elif topic == PEOPLE_SPEED_TOPIC:
            reasoner.input_people_speed(msg)
        elif topic == TF_SPEED_TOPIC:
            pass
        elif topic == TOUCH_SPEED_TOPIC:
            reasoner.input_touch_speed(msg)
        elif topic == REPLAN_REASON_TOPIC:
            reasoner.input_replan_reason(msg)
        elif topic == CURRENT_FRAME_TOPIC:
            reasoner.input_current_frame(msg)

        (duration, code) = reasoner.update()
        if prev_code != code:
            logger.debug("%.2f(%.2f), %s, %.2f", t/1e9, st, code.name, duration)
        prev_code = code
        stop_reason_filter.update(duration, code)
        (duration, code) = stop_reason_filter.summary()
        stop_reason_filter.conclude()

        if code:
            logger.info("### %.2f(%.2f), %s, %.2f, %s", t/1e9, st, code.name, duration,
                        "NAVIGATING" if reasoner.navigating else "NOT NAVIGATING")
    except:
        import traceback
        logger.error(traceback.format_exc())

