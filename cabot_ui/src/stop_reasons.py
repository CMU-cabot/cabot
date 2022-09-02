#!/usr/bin/env python

import logging
import os
import sys
from tf.transformations import quaternion_multiply, euler_from_quaternion

import rosbag
import tf_bag

import cabot_ui.geoutil
from cabot_ui.stop_reasoner import StopReasoner, StopReason

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

    for topic, msg, t in bag.read_messages(topics=all_topics):
        reasoner.update_time(t)
        code = None
        duration = 0

        if topic == ODOM_TOPIC:
            reasoner.input_odom(msg)
            (duration, code) = reasoner.update()

        if topic == EVENT_TOPIC:
            reasoner.input_event(msg)
            (duration, code) = reasoner.update()

        if topic in [NAVIGATE_TO_POSE_GOAL_TOPIC, LOCAL_PREFIX+NAVIGATE_TO_POSE_GOAL_TOPIC]:
            reasoner.input_goal_topic(msg)
            (duration, code) = reasoner.update()

        if topic in [NAVIGATE_TO_POSE_RESULT_TOPIC, LOCAL_PREFIX+NAVIGATE_TO_POSE_RESULT_TOPIC]:
            reasoner.input_result_topic(msg)
            (duration, code) = reasoner.update()

        if topic == CMD_VEL_TOPIC:
            reasoner.input_cmd_vel(msg)
            (duration, code) = reasoner.update()

        if topic == PEOPLE_SPEED_TOPIC:
            reasoner.input_people_speed(msg)
            (duration, code) = reasoner.update()

        if topic == TF_SPEED_TOPIC:
            (duration, code) = reasoner.update()

        if topic == TOUCH_SPEED_TOPIC:
            reasoner.input_touch_speed(msg)
            (duration, code) = reasoner.update()

        if topic == REPLAN_REASON_TOPIC:
            reasoner.input_replan_reason(msg)
            (duration, code) = reasoner.update()

        if topic == CURRENT_FRAME_TOPIC:
            reasoner.input_current_frame(msg)
            

        if not code:
            continue

        if not code in [StopReason.NO_NAVIGATION, StopReason.NO_TOUCH, StopReason.NOT_STOPPED, StopReason.STOPPED_BUT_UNDER_THRESHOLD]:
            if prev_code != code:
                logger.info("%.2f, %s, %.2f", t.to_sec(), str(code), duration)
                if callback:
                    callback(t, code, duration)
        elif prev_code != code:
            if prev_duration > 1.0:
                logger.debug("%.2f, %s-end, %.2f", t.to_sec(), str(prev_code), prev_duration)
            else:
                pass
                #logger.debug("%.2f, %s, %.2f", rospy.Time.now().to_sec(), str(prev_code), prev_duration)
        prev_code = code
        prev_duration = duration

    bag.close()

if __name__ == "__main__":
    read_from_bag(sys.argv[1])
