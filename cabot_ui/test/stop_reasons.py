#!/usr/bin/env python3

import logging
import os
import sys
from tf_transformations import quaternion_multiply, euler_from_quaternion

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import rosbag2_py

import cabot_ui.geoutil
from cabot_ui.stop_reasoner import StopReasoner, StopReason, StopReasonFilter

logging.basicConfig(format='%(name)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG if 'DEBUG' in os.environ else logging.INFO)

def get_rosbag_options(path, serialization_format='cdr'):
    storage_options = rosbag2_py.StorageOptions(uri=path, storage_id='sqlite3')

    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format=serialization_format,
        output_serialization_format=serialization_format)

    return storage_options, converter_options

def read_from_bag(bagfile, callback=None):
    storage_options, converter_options = get_rosbag_options(bagfile)
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)
    
    topic_types = reader.get_all_topics_and_types()
    type_map = {topic_types[i].name: topic_types[i].type for i in range(len(topic_types))}
    
    #tf_transformer = tf_bag.BagTfTransformer(bag)
    tf_transformer = None
    reasoner = StopReasoner(tf_transformer)

    prev_code = None
    prev_duration = 0

    ODOM_TOPIC="/cabot/odom_raw"
    EVENT_TOPIC="/cabot/event"
    CMD_VEL_TOPIC="/cmd_vel"
    PEOPLE_SPEED_TOPIC="/cabot/people_speed"
    TF_SPEED_TOPIC="/cabot/tf_speed"
    TOUCH_SPEED_TOPIC="/cabot/touch_speed_switched"
    NAVIGATE_TO_POSE_GOAL_TOPIC="/navigate_to_pose/goal"
    NAVIGATE_TO_POSE_RESULT_TOPIC="/navigate_to_pose/result"
    LOCAL_PREFIX="/local"
    REPLAN_REASON_TOPIC="/replan_reason"
    CURRENT_FRAME_TOPIC="/current_frame"

    all_topics = [
        ODOM_TOPIC,
        EVENT_TOPIC,
        CMD_VEL_TOPIC,
        PEOPLE_SPEED_TOPIC,
        TF_SPEED_TOPIC,
        TOUCH_SPEED_TOPIC,
        NAVIGATE_TO_POSE_GOAL_TOPIC,
        NAVIGATE_TO_POSE_RESULT_TOPIC,
        LOCAL_PREFIX+NAVIGATE_TO_POSE_GOAL_TOPIC,
        LOCAL_PREFIX+NAVIGATE_TO_POSE_RESULT_TOPIC,
        REPLAN_REASON_TOPIC,
        CURRENT_FRAME_TOPIC,
    ]

    stop_reason_filter = StopReasonFilter(["NO_NAVIGATION", "NOT_STOPPED", "NO_TOUCH", "STOPPED_BUT_UNDER_THRESHOLD"])

    storage_filter = rosbag2_py.StorageFilter(topics=all_topics)
    reader.set_filter(storage_filter)
    
    while reader.has_next():
        (topic, msg_data, t) = reader.read_next()
        msg_type = get_message(type_map[topic])
        msg = deserialize_message(msg_data, msg_type)
        reasoner.update_time(t)

        code = None
        duration = 0

        if topic == ODOM_TOPIC:
            reasoner.input_odom(msg)
        elif topic == EVENT_TOPIC:
            reasoner.input_event(msg)
        elif topic in [NAVIGATE_TO_POSE_GOAL_TOPIC, LOCAL_PREFIX+NAVIGATE_TO_POSE_GOAL_TOPIC]:
            reasoner.input_goal_topic(msg)
        elif topic in [NAVIGATE_TO_POSE_RESULT_TOPIC, LOCAL_PREFIX+NAVIGATE_TO_POSE_RESULT_TOPIC]:
            reasoner.input_result_topic(msg)
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
        stop_reason_filter.update(duration, code)
        (duration, code) = stop_reason_filter.summary()
        stop_reason_filter.conclude()

        if code:
            logger.info("%.2f, %s, %.2f", t/1e9, code.name, duration)

if __name__ == "__main__":
    read_from_bag(sys.argv[1])
