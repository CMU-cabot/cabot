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

import logging
import multiprocessing
import signal
import sys
import struct
import termios
import traceback
from time import sleep, time

from serial import Serial, SerialException

import rospy

from cabot.util import setInterval
from sensor_msgs.msg import Imu, FluidPressure, Temperature
from std_msgs.msg import Bool, UInt8, UInt8MultiArray, Int8, Int16, Float32, Float32MultiArray
from std_srvs.srv import SetBool, SetBoolResponse
from diagnostic_updater import Updater, DiagnosticTask, HeaderlessTopicDiagnostic, FrequencyStatusParam
from diagnostic_msgs.msg import DiagnosticStatus


# global variables
imu_last_topic_time = None
imu_pub = None
btn_pubs = []
NUMBER_OF_BUTTONS = 5
btn_sub = None

def imu_callback(data):
    global imu_last_topic_time
    ## discard possible corrupted data
    count = 0
    data2 = [struct.unpack('f', data[i*4:(i+1)*4])[0] for i in range(0, 12)]

    for i in range(2, 12):
        if data2[i] == 0:
            count += 1
    if count > 3:
        return

    imu_msg = Imu()
    imu_msg.orientation_covariance[0] = 0.1
    imu_msg.orientation_covariance[4] = 0.1
    imu_msg.orientation_covariance[8] = 0.1

    # convert float(32) to int(32)
    imu_msg.header.stamp.set(struct.unpack('i', struct.pack('f', data2[0]))[0]
                             ,struct.unpack('i', struct.pack('f', data2[1]))[0])
    if imu_last_topic_time is not None:
        if imu_last_topic_time > imu_msg.header.stamp:
            rospy.logerr("IMU timestamp is not consistent, drop a message\n"+
                         "last imu time:%.2f > current imu time:%.2f",
                           imu_last_topic_time.to_sec(), imu_msg.header.stamp.to_sec())
            return

    imu_msg.header.frame_id = "imu_frame"
    imu_last_topic_time = imu_msg.header.stamp
    imu_msg.orientation.x = data2[2]
    imu_msg.orientation.y = data2[3]
    imu_msg.orientation.z = data2[4]
    imu_msg.orientation.w = data2[5]
    imu_msg.angular_velocity.x = data2[6]
    imu_msg.angular_velocity.y = data2[7]
    imu_msg.angular_velocity.z = data2[8]
    imu_msg.linear_acceleration.x = data2[9]
    imu_msg.linear_acceleration.y = data2[10]
    imu_msg.linear_acceleration.z = data2[11]

    imu_pub.publish(imu_msg)


# touch speed active mode
# True:  Touch - go,    Not Touch - no go
# False: Touch - no go, Not Touch - go
touch_speed_active_mode = True
touch_speed_max = 2.0
touch_speed_max_inactive = 0.5
touch_speed_switched_pub = None

def touch_callback(msg):
    touch_speed_msg = Float32()
    if touch_speed_active_mode:
        touch_speed_msg.data = touch_speed_max_speed if msg.data else 0.0
        touch_speed_switched_pub.publish(touch_speed_msg)
    else:
        touch_speed_msg.data = 0.0 if msg.data else touch_speed_max_speed_inactive
        touch_speed_switched_pub.publish(touch_speed_msg)

def btn_callback(msg):
    for i in range(0, NUMBER_OF_BUTTONS):
        temp = Bool()
        temp.data = (msg.data >> i) & 0x01
        btn_pubs[i].publish(temp)

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

class TopicCheckTask(HeaderlessTopicDiagnostic):
    def __init__(self, updater, name, topic, topic_type, freq, callback=lambda x:x):
        super().__init__(name, updater, FrequencyStatusParam({'min':freq, 'max':freq}, 0.1, 2))
        self.sub = rospy.Subscriber(topic, topic_type, self.topic_callback)
        self.callback = callback

    def topic_callback(self, msg):
        global topic_alive
        self.callback(msg)
        self.tick()
        topic_alive = time()

class CheckConnectionTask(DiagnosticTask):
    def __init__(self, name):
        super().__init__(name)

    def run(self, stat):
        global port, topic_alive
        if client is None:
            if error_msg is None:
                stat.summary(DiagnosticStatus.WARN, "connecting")
            else:
                stat.summary(DiagnosticStatus.ERROR, error_msg)
        else:
            if topic_alive and time() - topic_alive > 5:
                rospy.logerr("connected but no message comming")
                stat.summary(DiagnosticStatus.ERROR, "connected but no message comming")
                port.close()
                topic_alive = None
            else:
                stat.summary(DiagnosticStatus.OK, "working")


from cabot.arduino_serial import CaBotArduinoSerialDelegate, CaBotArduinoSerial
stopped = False
class ROSDelegate(CaBotArduinoSerialDelegate):
    def __init__(self):
        self.owner = None

        self.touch_raw_pub = rospy.Publisher("/touch_raw", Int16, queue_size=10)
        self.touch_pub = rospy.Publisher("/touch", Int16, queue_size=10)
        self.button_pub = rospy.Publisher("/pushed", Int8, queue_size=10)
        self.imu_last_topic_time = None
        self.imu_pub = rospy.Publisher("/imu", Imu, queue_size=10)
        self.calibration_pub = rospy.Publisher("/calibration", UInt8MultiArray, queue_size=10)
        self.pressure_pub = rospy.Publisher("/pressure", FluidPressure, queue_size=10)
        self.temperature_pub = rospy.Publisher("/temperature", Temperature, queue_size=10)

        self.vib1_sub = rospy.Subscriber("/vibrator1", UInt8, self.vib_callback(0x20))
        self.vib2_sub = rospy.Subscriber("/vibrator2", UInt8, self.vib_callback(0x21))
        self.vib3_sub = rospy.Subscriber("/vibrator3", UInt8, self.vib_callback(0x22))
        self.vib4_sub = rospy.Subscriber("/vibrator4", UInt8, self.vib_callback(0x23))

    def vib_callback(self, cmd):
        def callback(msg):
            data = bytearray()
            data.append(msg.data)
            self.owner.send_command(cmd, data, 1)
        return callback

    def system_time(self):
        now = rospy.Time.now()
        return (now.secs, now.nsecs)

    def stopped(self):
        global stopped
        stopped = True

    def log(self, level, text):
        if level == logging.INFO:
            rospy.loginfo(text)
        if level == logging.WARN:
            rospy.logwarn(text)
        if level == logging.ERROR:
            rospy.logerr(text)

    def log_throttle(self, level, interval, text):
        if level == logging.INFO:
            rospy.loginfo_throttle(interval, text)
        if level == logging.WARN:
            rospy.logwarn_throttle(interval, text)
        if level == logging.ERROR:
            rospy.logerr_throttle(interval, text)

    def get_param(self, name, callback):
        val = rospy.get_param(name, None)
        if val:
            callback(val)
        else:
            callback([])

    def publish(self, cmd, data):
        #rospy.loginfo("%x: %d", cmd, int.from_bytes(data, "little"))
        # rospy.loginfo("%x: %s", cmd, str(data));
        #self.log_throttle(logging.INFO, 1, "got data %x"%(cmd))
        if cmd == 0x10:  # touch
            msg = Int16()
            msg.data = int.from_bytes(data, 'little')
            self.touch_pub.publish(msg)
        if cmd == 0x11:  # touch_raw
            msg = Int16()
            msg.data = int.from_bytes(data, 'little')
            self.touch_raw_pub.publish(msg)
        if cmd == 0x12:  # buttons
            msg = Int8()
            msg.data = int.from_bytes(data, 'little')
            self.button_pub.publish(msg)
        if cmd == 0x13:  # imu
            imu_callback(data)
        if cmd == 0x14:  # calibration
            msg = UInt8MultiArray()
            msg.data = data
            self.calibration_pub.publish(msg)
        if cmd == 0x15:  # pressure
            msg = FluidPressure()
            msg.fluid_pressure = struct.unpack('f', data)[0]
            msg.variance = 0;
            msg.header.stamp = rospy.Time.now();
            msg.header.frame_id = "bmp_frame";
            self.pressure_pub.publish(msg)
        if cmd == 0x16:  # temperature
            msg = Temperature()
            msg.temperature = struct.unpack('f', data)[0]
            msg.variance = 0;
            msg.header.stamp = rospy.Time.now();
            msg.header.frame_id = "bmp_frame";
            self.temperature_pub.publish(msg)

if __name__=="__main__":
    rospy.init_node("cabot_serial_node")
    rospy.loginfo("CABOT ROS Serial Python Node")

    ## IMU
    imu_pub = rospy.Publisher("imu", Imu, queue_size=10)

    ## touch speed control
    touch_speed_max_speed = rospy.get_param('~touch_speed_max', 2.0)
    touch_speed_max_speed_inactive = rospy.get_param('~touch_speed_max_inactive', 0.5)
    touch_speed_switched_pub = rospy.Publisher("touch_speed_switched", Float32, queue_size=10)
    set_touch_speed_active_mode_srv = rospy.Service("set_touch_speed_active_mode", SetBool, set_touch_speed_active_mode)

    ## button
    for i in range(0, NUMBER_OF_BUTTONS):
        btn_pubs.append(rospy.Publisher("pushed_%d"%(i+1), Bool, queue_size=10))
    btn_sub = rospy.Subscriber("pushed", Int8, btn_callback)

    ## Diagnostic Updater
    updater = Updater()
    TopicCheckTask(updater, "IMU", "imu", Imu, 100)
    TopicCheckTask(updater, "Touch Sensor", "touch", Int16, 50, touch_callback)
    for i in range(1, NUMBER_OF_BUTTONS+1):
        TopicCheckTask(updater, "Push Button %d"%(i), "pushed_%d"%(i), Bool, 50)
    TopicCheckTask(updater, "Pressure", "pressure", FluidPressure, 2)
    TopicCheckTask(updater, "Temperature", "temperature", Temperature, 2)
    rospy.Timer(rospy.Duration(0.5), lambda e: updater.update())
    updater.add(CheckConnectionTask("Serial Connection"))

    ## add the following line into /etc/udev/rules.d/10-local.rules
    ## ACTION=="add", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6015", SYMLINK+="ttyCABOT"
    port_name = rospy.get_param('~port','/dev/ttyCABOT')
    #port_names = [port_name, '/dev/ttyUSB0', '/dev/ttyUSB1', '/dev/ttyUSB2']
    port_names = [port_name]
    port_index = 0
    baud = int(rospy.get_param('~baud','115200'))


    # for systems where pyserial yields errors in the fcntl.ioctl(self.fd, TIOCMBIS, \
    # TIOCM_DTR_str) line, which causes an IOError, when using simulated port
    fix = rospy.get_param('~fix_pyserial_for_test', False)
    sleep_time=3


    def handler(signum, frame):
        signame = signal.Signals(signum).name
        print(f'Signal handler called with signal {signame} ({signum})')
        raise SystemExit()

    # Set the signal handler and a 5-second alarm
    signal.signal(signal.SIGINT, handler)

    error_msg = None
    delegate = ROSDelegate()
    while not rospy.is_shutdown():
        try:
            client = None
            port = None
            port_name = port_names[port_index]
            port_index = (port_index + 1) % len(port_names)
            rospy.loginfo("Connecting to %s at %d baud" % (port_name,baud) )
            while not rospy.is_shutdown():
                try:
                    if fix:
                        port = Serial(port_name, baud, timeout=5, write_timeout=10, rtscts=True, dsrdtr=True)
                    else:
                        port = Serial(port_name, baud, timeout=5, write_timeout=10)
                    break
                except SerialException as e:
                    error_msg = str(e)
                    rospy.logerr("%s", e)
                    sleep(3)

            client = CaBotArduinoSerial(port, baud)
            delegate.owner = client
            client.delegate = delegate
            updater.setHardwareID(port_name)
            topic_alive = None
            client.start()
            while client.is_alive:
                sleep(0.0001)
        except KeyboardInterrupt as e:
            client.stop()
            rospy.loginfo("KeyboardInterrupt")
            rospy.signal_shutdown("user interrupted")
            break
        except SerialException as e:
            error_msg = str(e)
            rospy.logerr(e)
            sleep(sleep_time)
            continue
        except OSError as e:
            error_msg = str(e)
            rospy.logerr(e)
            traceback.print_exc(file=sys.stdout)
            sleep(sleep_time)
            continue
        except IOError as e:
            error_msg = str(e)
            rospy.logerr("try to reconnect usb")
            sleep(sleep_time)
            continue
        except termios.error as e:
            error_msg = str(e)
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
