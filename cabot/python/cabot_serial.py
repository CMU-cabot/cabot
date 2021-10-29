#!/usr/bin/env python

# Copyright (c) 2020  Carnegie Mellon University
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

import termios
import traceback

import rospy
from rosserial_python import SerialClient, RosSerialServer
from serial import SerialException
from time import sleep
import multiprocessing
from cabot.util import setInterval
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Imu
from std_msgs.msg import Int16, Float32
from std_srvs.srv import SetBool, SetBoolResponse

import sys
import struct

imu_pub = None
last_imu_time = None

# touch speed active mode
# True:  Touch - go,    Not Touch - no go
# False: Touch - no go, Not Touch - go
touch_speed_active_mode = True
touch_speed_max = 2.0
touch_speed_max_inactive = 0.5
touch_speed_switched_pub = None

def callback(msg):
    global last_imu_time
    
    imu_msg = Imu()
    imu_msg.orientation_covariance[0] = 0.1
    imu_msg.orientation_covariance[4] = 0.1
    imu_msg.orientation_covariance[8] = 0.1

    # convert float(32) to int(32)
    imu_msg.header.stamp.set(struct.unpack('i', struct.pack('f', msg.data[0]))[0]
                             ,struct.unpack('i', struct.pack('f', msg.data[1]))[0])
    if last_imu_time is not None:
        if last_imu_time > imu_msg.header.stamp:
            rospy.logerr("IMU timestamp is not consistent, drop a message\n"+
                         "last imu time:%.2f > current imu time:%.2f",
                           last_imu_time.to_sec(), imu_msg.header.stamp.to_sec())
            return
    last_imu_time = imu_msg.header.stamp
    imu_msg.header.frame_id = "imu_frame"

    count = 0
    for i in range(2, 12):
        if msg.data[i] == 0:
            count += 1
    if count > 3:
        return

    imu_msg.orientation.x = msg.data[2]
    imu_msg.orientation.y = msg.data[3]
    imu_msg.orientation.z = msg.data[4]
    imu_msg.orientation.w = msg.data[5]

    imu_msg.angular_velocity.x = msg.data[6]
    imu_msg.angular_velocity.y = msg.data[7]
    imu_msg.angular_velocity.z = msg.data[8]

    imu_msg.linear_acceleration.x = msg.data[9]
    imu_msg.linear_acceleration.y = msg.data[10]
    imu_msg.linear_acceleration.z = msg.data[11]

    imu_pub.publish(imu_msg)

def touch_callback(msg):
    touch_speed_msg = Float32()
    if touch_speed_active_mode:
        touch_speed_msg.data = touch_speed_max if msg.data else 0.0
        touch_speed_switched_pub.publish(touch_speed_msg)
    else:
        touch_speed_msg.data = 0.0 if msg.data else touch_speed_max_inactive
        touch_speed_switched_pub.publish(touch_speed_msg)

def set_touch_speed_active_mode(msg):
    global touch_speed_active_mode
    touch_speed_active_mode = msg.data

    resp = SetBoolResponse()
    if touch_speed_active_mode:
        resp.message = "touch speed active mode = True"
    else:
        resp.message = "touch speed active mode = False"
    resp.success = True
    return resp

def startSerialNode():
    rospy.init_node("cabot_serial_node")
    rospy.loginfo("CABOT ROS Serial Python Node")

    rospy.Subscriber("imu_raw", Float32MultiArray, callback)
    global imu_pub
    imu_pub = rospy.Publisher("imu", Imu, queue_size=10)

    ## touch speed control
    global touch_speed_switched_pub, touch_speed_max, touch_speed_max_inactive
    touch_speed_max = rospy.get_param('~touch_speed_max', 2.0)
    touch_speed_max_inactive = rospy.get_param('~touch_speed_max_inactive', 0.5)
    rospy.Subscriber("touch", Int16, touch_callback)
    touch_speed_switched_pub = rospy.Publisher("touch_speed_switched", Float32, queue_size=10)
    set_touch_speed_active_mode_srv = rospy.Service("set_touch_speed_active_mode", SetBool, set_touch_speed_active_mode)

    ## add the following line into /etc/udev/rules.d/10-local.rules
    ## ACTION=="add", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6015", SYMLINK+="ttyCABOT"
    port_name = rospy.get_param('~port','/dev/ttyCABOT')
    #port_names = [port_name, '/dev/ttyUSB0', '/dev/ttyUSB1', '/dev/ttyUSB2']
    port_names = [port_name]
    port_index = 0
    baud = int(rospy.get_param('~baud','1000000'))

    # for systems where pyserial yields errors in the fcntl.ioctl(self.fd, TIOCMBIS, \
    # TIOCM_DTR_str) line, which causes an IOError, when using simulated port
    fix = rospy.get_param('~fix_pyserial_for_test', False)
    sleep_time=3
    
    while not rospy.is_shutdown():
        try:
            port_name = port_names[port_index]
            port_index = (port_index + 1) % len(port_names)
            rospy.loginfo("Connecting to %s at %d baud" % (port_name,baud) )
            client = SerialClient(port_name, baud, fix_pyserial_for_test=fix)
            client.run()
        except KeyboardInterrupt as e:
            rospy.loginfo("KeyboardInterrupt")
            rospy.signal_shutdown("user interrupted")
            break
        except SerialException as e:
            rospy.logerr(e)
            sleep(sleep_time)
            continue
        except OSError as e:
            rospy.logerr(e)
            traceback.print_exc(file=sys.stdout)
            sleep(sleep_time)
            continue
        except IOError as e:
            rospy.logerr("try to reconnect usb")
            sleep(sleep_time)
            continue
        except termios.error as e:
            rospy.logerr("connection disconnected")
            sleep(sleep_time)
            continue
        except SystemExit as e:
            break
        except:
            rospy.logerr(sys.exc_info()[0])
            traceback.print_exc(file=sys.stdout)
            rospy.signal_shutdown()
            sys.exit()

if __name__=="__main__":
    startSerialNode()
