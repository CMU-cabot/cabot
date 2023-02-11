#!/usr/bin/env python

import logging
import os
import sys
from tf.transformations import quaternion_multiply, euler_from_quaternion

import rosbag
import tf_bag

import cabot_ui.geoutil
from cabot_ui.stop_reasoner import StopReasoner, StopReason, StopReasonFilter

logging.basicConfig(format='%(name)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG if 'DEBUG' in os.environ else logging.INFO)

def read_from_bag(bagfile, callback=None):
    bag = rosbag.Bag(bagfile)
    tf_transformer = tf_bag.BagTfTransformer(bag)
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

    stop_reason_filter = StopReasonFilter()
    for topic, msg, t in bag.read_messages(topics=all_topics):
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
            logger.info("%.2f, %s, %.2f", t.to_sec(), code.name, duration)

    bag.close()

if __name__ == "__main__":
    read_from_bag(sys.argv[1])
