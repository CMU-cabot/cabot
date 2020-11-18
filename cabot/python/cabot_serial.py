#!/usr/bin/env python

# Copyright 2020 Carnegie Mellon University
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

import sys

def startSerialNode():
    rospy.init_node("cabot_serial_node")
    rospy.loginfo("CABOT ROS Serial Python Node")

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
