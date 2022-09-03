#!/usr/bin/env python

import logging
import os
import sys
from tf.transformations import quaternion_multiply, euler_from_quaternion

import rospy
import tf
import nav_msgs.msg
import std_msgs.msg
import geometry_msgs.msg
import nav2_msgs.msg
import cabot_msgs.msg
import people_msgs.msg
import threading

import cabot_ui.geoutil
from cabot_ui.stop_reasoner import StopReasonFilter, StopReasoner, StopReason, StopReasonFilter
from cabot_ui.event import NavigationEvent

reasoner = None
stop_reason_pub = None
event_pub = None

def main():
    global reasoner, stop_reason_pub, event_pub
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

    rospy.init_node("stop_reason_node")
    tf_listener = tf.TransformListener()
    reasoner = StopReasoner(tf_listener)

    stop_reason_pub = rospy.Publisher("/stop_reason", cabot_msgs.msg.StopReason, queue_size=10)
    event_pub = rospy.Publisher("/cabot/event", std_msgs.msg.String, queue_size=10)

    rospy.Subscriber(ODOM_TOPIC, nav_msgs.msg.Odometry, odom_callback)
    rospy.Subscriber(EVENT_TOPIC, std_msgs.msg.String, event_callback)
    rospy.Subscriber(CMD_VEL_TOPIC, geometry_msgs.msg.Twist, cmd_vel_callback)
    rospy.Subscriber(PEOPLE_SPEED_TOPIC, std_msgs.msg.Float32, people_speed_callback)
    rospy.Subscriber(TF_SPEED_TOPIC, std_msgs.msg.Float32, tf_speed_callback)
    rospy.Subscriber(TOUCH_SPEED_TOPIC, std_msgs.msg.Float32, touch_speed_callback)
    rospy.Subscriber(NAVIGATE_TO_POSE_GOAL_TOPIC, nav2_msgs.msg.NavigateToPoseActionGoal, goal_callback)
    rospy.Subscriber(NAVIGATE_TO_POSE_RESULT_TOPIC, nav2_msgs.msg.NavigateToPoseActionResult, result_callback)
    rospy.Subscriber(LOCAL_PREFIX+NAVIGATE_TO_POSE_GOAL_TOPIC, nav2_msgs.msg.NavigateToPoseActionGoal, goal_callback)
    rospy.Subscriber(LOCAL_PREFIX+NAVIGATE_TO_POSE_RESULT_TOPIC, nav2_msgs.msg.NavigateToPoseActionResult, result_callback)
    rospy.Subscriber(REPLAN_REASON_TOPIC, people_msgs.msg.Person, replan_reason_callback)
    rospy.Subscriber(CURRENT_FRAME_TOPIC, std_msgs.msg.String, current_frame_callback)

    rospy.spin()

lock = threading.Lock()
stop_reason_filter = StopReasonFilter()

def update():
    with lock:
        global prev_code, prev_duration
        (duration, code) = reasoner.update()
        stop_reason_filter.update(duration, code)
        (duration, code) = stop_reason_filter.event()
        if code:
            msg = cabot_msgs.msg.StopReason()
            msg.header.stamp = rospy.Time.now()
            msg.reason = code.name
            msg.duration = duration
            stop_reason_pub.publish(msg)
        (duration, code) = stop_reason_filter.summary()
        if code:
            event = NavigationEvent("stop-reason", code.name)
            msg = std_msgs.msg.String()
            msg.data = str(event)
            event_pub.publish(msg)
            rospy.loginfo("%.2f, %s, %.2f", rospy.Time.now().to_sec(), code.name, duration)
        stop_reason_filter.conclude()

def odom_callback(msg):
    reasoner.input_odom(msg)
    update()

def event_callback(msg):
    reasoner.input_event(msg)
    update()

def goal_callback(msg):
    reasoner.input_goal_topic(msg)
    update()

def result_callback(msg):
    reasoner.input_result_topic(msg)
    update()

def cmd_vel_callback(msg):
    reasoner.input_cmd_vel(msg)
    update()

def people_speed_callback(msg):
    reasoner.input_people_speed(msg)
    update()

def tf_speed_callback(msg):
    update()

def touch_speed_callback(msg):
    reasoner.input_touch_speed(msg)
    update()

def replan_reason_callback(msg):
    reasoner.input_replan_reason(msg)
    update()

def current_frame_callback(msg):
    reasoner.input_current_frame(msg)


if __name__ == "__main__":
    main()
