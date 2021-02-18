#!/usr/bin/env python

# Copyright (c) 2020 Carnegie Mellon University
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

import math

import rospy
import tf2_ros
import std_msgs.msg
import geometry_msgs.msg
import sensor_msgs.msg
import nav_msgs.msg
from tf_conversions import transformations

from cabot.util import setInterval

clutch_state=False

motorPub = None
wheelRadius = 0.045
differential = 0.0825
encoderCountPerRound = 1000.0 * 26.0
encoderHz = 100.0
unit = 0.1

lspeed = 0
tspeed = 0
targetLspeed = 0
targetTspeed = 0
lastMotorData = None

interval = 0.1
motor_speed = 2661 / 1.2539 # maximum speed in m/s
motor_publishing = False


@setInterval(0.5, 1)
def checkStartMotorPublisher():
    global motor_publishing
    if motor_publishing:
        return
    motor_publishing = True
    rospy.loginfo("Start publishing motor speed")
    publishMotorSpeed()

@setInterval(interval)
def publishMotorSpeed():
    global lspeed, tspeed, lastMotorData
    
    eps = motor_speed*0.5*interval

    if abs(lspeed - targetLspeed) < eps:
        lspeed = targetLspeed
    else:
        lspeed += eps if lspeed < targetLspeed else -eps
    
    if abs(tspeed - targetTspeed) < eps:
        tspeed = targetTspeed
    else:
        tspeed += eps if tspeed < targetTspeed else -eps
        
    motorData = std_msgs.msg.Int16MultiArray()
    motorData.data = [lspeed-tspeed, lspeed+tspeed]


    if motorPub is not None:
        if lastMotorData is None or lastMotorData.data[0] != motorData.data[0] or lastMotorData.data[1] != motorData.data[1]:
            #print motorData.data, lspeed, targetLspeed, tspeed, targetTspeed
            motorPub.publish(motorData)
            lastMotorData = motorData

factor = 1.0
def callback(data):
    global targetLspeed, targetTspeed
    
    linear = data.linear.x
    theta = data.angular.z

    pi2 = 2.0*math.pi
    #ls = -1.0 * (linear / (pi2 * wheelRadius)) * 60 / unit
    #ts = -1.0 * ((theta / pi2 * differential) / (wheelRadius)) * 60 / unit

    ls = -1.0 * linear;
    ts = -1.0 * theta * differential;

    motor = 1 / (pi2 * wheelRadius) * 60 / unit

    targetLspeed = ls * factor * motor;
    targetTspeed = ts * factor * motor;

    #rospy.loginfo("%.2f %.2f, %.2f %.2f, %.2f %.2f" % (linear, theta, targetLspeed, targetTspeed, lspeed, tspeed))

lastVoltageTime = 0
        
def voltageCallback(data):
    global lastVoltageTime
    checkStartMotorPublisher()
    
    now = rospy.get_rostime().to_sec()
    if (now - lastVoltageTime) > 60:
        rospy.loginfo("Voltage = %.1f" % (data.data/1000))
        lastVoltageTime = now

orientation = 0
diffL = None
diffR = None
enc1 = None
enc2 = None
count1 = 0
count2 = 0
seq = 0
robotx = 0
roboty = 0
robotz = 0
# fixed covariance
cov = [0.01, 0, 0, 0, 0, 0,
       0, 0.01, 0, 0, 0, 0,
       0, 0, 0.01, 0, 0, 0,
       0, 0, 0, 0.10, 0, 0,
       0, 0, 0, 0, 0.10, 0,
       0, 0, 0, 0, 0, 0.10]

# value is opposite for diffL
def encoder1Callback(data):
    global enc1, diffL, count1
    if enc1 is None:
        enc1 = data.data
        return
    count1 += 1
    if count1 == 5:
        diffL = - (data.data - enc1)
        check_diff()
        enc1 = data.data
        count1 = 0

def encoder2Callback(data):
    global enc2, diffR, count2
    if enc2 is None:
        enc2 = data.data
        return
    count2 += 1
    if count2 == 5:
        diffR = data.data - enc2
        check_diff()
        enc2 = data.data
        count2 = 0

vel_x = 0
ang_z = 0
last_odom = None

def check_diff():
    global seq, robotx, roboty, orientation, diffL, diffR
    if diffR is None or diffL is None:
        return
    
    #rospy.loginfo('diff %f, %f'%(diffL, diffR))
    diffL = diffL / encoderCountPerRound * 2 * math.pi * wheelRadius
    diffR = diffR / encoderCountPerRound * 2 * math.pi * wheelRadius
    #rospy.loginfo('diff %f, %f'%(diffL, diffR))
    
    delta = (diffR - diffL) / 2 / differential
    
    dx = (diffL + diffR) / 2 * math.cos(orientation + delta)
    dy = (diffL + diffR) / 2 * math.sin(orientation + delta)

    robotx += dx
    roboty += dy

    orientation += delta

    odom = nav_msgs.msg.Odometry()
    odom.header.seq = seq
    odom.header.stamp = rospy.get_rostime()
    odom.header.frame_id = 'odom'
    odom.child_frame_id = 'base_footprint'
    odom.pose.pose.position.x = robotx
    odom.pose.pose.position.y = roboty
    odom.pose.pose.position.z = robotz
    q = transformations.quaternion_from_euler(0, 0, orientation)
    odom.pose.pose.orientation = geometry_msgs.msg.Quaternion(*q)
    
    odom.pose.covariance = cov

    sign = 1 if (diffL+diffR) > 0 else -1
    global vel_x, ang_z
    vel_x = vel_x * 0.5 + (sign * abs(dx) * encoderHz / 5) * 0.5
    odom.twist.twist.linear.x = vel_x
    odom.twist.twist.linear.y = 0
    odom.twist.twist.linear.z = 0
    odom.twist.twist.angular.x = 0
    odom.twist.twist.angular.y = 0
    if last_imu is not None:
        ang_z = ang_z * 0.5 + last_imu.angular_velocity.z * 0.5
        odom.twist.twist.angular.z = ang_z

    odom.twist.covariance = cov
    global last_odom
    last_odom = odom
    odomPub.publish(odom)

    #replaced with robot pose ekf
    #publishTransform(odom, "odom", "base_footprint")
                
    #rospy.loginfo(odom)
    
    seq += 1
    diffL = None
    diffR = None

def publishTransform(data, odomInput, tfOutput):
    tfBroadcaster = tf2_ros.TransformBroadcaster()

    t = geometry_msgs.msg.TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = odomInput
    t.child_frame_id = tfOutput
    t.transform.translation = data.pose.pose.position
    t.transform.rotation = data.pose.pose.orientation
    
    tfBroadcaster.sendTransform(t)


d2r = math.pi / 180.0
last_imu = None

def imuCallback(data, args):
    pub = args
    data.header.stamp = rospy.get_rostime()
    data.angular_velocity.x *= d2r
    data.angular_velocity.y *= d2r
    data.angular_velocity.z *= d2r
    global last_imu
    last_imu = data
    pub.publish(data)

def ekfCallback(msg):
    if last_odom is not None:
        last_odom.pose = msg.pose
        odom2Pub.publish(last_odom)

if __name__ == '__main__':
    rospy.init_node("cabot_mobile_base")
    rospy.loginfo("init")

    cmdVelInput = rospy.get_param("~cmd_vel_topic", "/cmd_vel")
    motorOutput = rospy.get_param("~motor_topic", "/motor_speed")
    voltageInput = rospy.get_param("~voltage_topic", "/voltage")
    
    encoder1Input = rospy.get_param("~encoder1_topic", "/encoder1")
    encoder2Input = rospy.get_param("~encoder2_topic", "/encoder2")
    odomOutput = rospy.get_param("~odom_topic", "/odom")
    ekfInput = rospy.get_param("~ekf_topic", "/robot_pose_ekf/odom_combined")
    odom2Output = rospy.get_param("~odom2_topic", "/odom2")
    
    motorPub = rospy.Publisher(motorOutput, std_msgs.msg.Int16MultiArray, queue_size=1)
    odomPub = rospy.Publisher(odomOutput, nav_msgs.msg.Odometry, queue_size=100)
    odom2Pub = rospy.Publisher(odom2Output, nav_msgs.msg.Odometry, queue_size=100)    
    
    rospy.Subscriber(cmdVelInput, geometry_msgs.msg.Twist, callback, None)
    rospy.Subscriber(voltageInput, std_msgs.msg.UInt16, voltageCallback, None)
    rospy.Subscriber(encoder1Input, std_msgs.msg.Int32, encoder1Callback, None)
    rospy.Subscriber(encoder2Input, std_msgs.msg.Int32, encoder2Callback, None)
    rospy.Subscriber(ekfInput, geometry_msgs.msg.PoseWithCovarianceStamped, ekfCallback, None)
    
    imuInput = rospy.get_param("~imu_input", "/imu")
    imuOutput = rospy.get_param("~imu_output", "/imu2")
    imuPub = rospy.Publisher(imuOutput, sensor_msgs.msg.Imu, queue_size=100)
    rospy.Subscriber(imuInput, sensor_msgs.msg.Imu, imuCallback, imuPub)
    
rospy.spin()


rospy.spin()
