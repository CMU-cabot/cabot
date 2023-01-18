#!/usr/bin/env python3

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

import rclpy

from cabot.util import setInterval
from sensor_msgs.msg import Imu, FluidPressure, Temperature
from std_msgs.msg import Bool, UInt8, UInt8MultiArray, Int8, Int16, Float32, Float32MultiArray, String
from std_srvs.srv import SetBool
from diagnostic_updater import Updater, DiagnosticTask, HeaderlessTopicDiagnostic, FrequencyStatusParam
from diagnostic_msgs.msg import DiagnosticStatus


# global variables
topic_alive = None

class TopicCheckTask(HeaderlessTopicDiagnostic):
    def __init__(self, updater, node, name, topic, topic_type, freq, callback=lambda x:x):
        super().__init__(name, updater, FrequencyStatusParam({'min':freq, 'max':freq}, 1.0, 2))
        self.sub = node.create_subscription(topic_type, topic, self.topic_callback, 10)
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
                logger.error("connected but no message comming")
                stat.summary(DiagnosticStatus.ERROR, "connected but no message comming")
                port.close()
                topic_alive = None
            else:
                stat.summary(DiagnosticStatus.OK, "working")
        return stat


from cabot.arduino_serial import CaBotArduinoSerialDelegate, CaBotArduinoSerial
stopped = False
class ROSDelegate(CaBotArduinoSerialDelegate):
    def __init__(self):
        self.owner = None
        self.wifi_pub = node.create_publisher(String, "wifi_scan_str", 50)

    def vib_callback(self, cmd):
        def callback(msg):
            data = bytearray()
            data.append(msg.data)
            self.owner.send_command(cmd, data, 1)
        return callback

    def system_time(self):
        now = node.get_clock().now().nanoseconds

        return (int(now / 1000000000), int(now % 1000000000))

    def stopped(self):
        global stopped
        stopped = True

    def log(self, level, text):
        if level == logging.INFO:
            logger.info(text)
        if level == logging.WARN:
            logger.warn(text)
        if level == logging.DEBUG:
            logger.debug(text)
        if level == logging.ERROR:
            logger.error(text)

    def log_throttle(self, level, interval, text):
        if level == logging.INFO:
            logger.info(text, throttle_duration_sec=interval)
        if level == logging.WARN:
            logger.warn(text, throttle_duration_sec=interval)
        if level == logging.DEBUG:
            logger.debug(text, throttle_duration_sec=interval)
        if level == logging.ERROR:
            logger.error(text, throttle_duration_sec=interval)

    def get_param(self, name, callback):
        if node.has_parameter(name):
            val = node.get_parameter(name).value
            if val is not None:
                callback(val)
                return
        callback([])

    def publish(self, cmd, data):
        if cmd == 0x20:  # wifi
            msg = String()
            msg.data = data.decode()
            self.wifi_pub.publish(msg)
        else:
            return


def shutdown_hook(signal_num, frame):
    print("shutdown_hook cabot_serial.py (wireless scanner)")
    sys.exit(0)


signal.signal(signal.SIGINT, shutdown_hook)


if __name__=="__main__":
    rclpy.init()
    node = rclpy.create_node("cabot_serial_node")
    logger = node.get_logger()
    logger.info("CABOT ROS Serial Python Node")

    node.declare_parameter('port', '/dev/ttyCABOT')
    node.declare_parameter('baud', 115200)

    ## Diagnostic Updater
    updater = Updater(node)
    TopicCheckTask(updater, node, "WiFi", "wifi_scan_str", String, 50)
    updater.add(CheckConnectionTask("Serial Connection"))

    ## add the following line into /etc/udev/rules.d/10-local.rules
    port_name = node.get_parameter('port').value
    port_names = [port_name]
    port_index = 0
    baud = node.get_parameter('baud').value

    sleep_time=3
    error_msg = None
    delegate = ROSDelegate()

    def run():
        global port_index, port_name, client
        while rclpy.ok():
            try:
                client = None
                port = None
                port_name = port_names[port_index]
                port_index = (port_index + 1) % len(port_names)
                logger.info("Connecting to %s at %d baud" % (port_name,baud) )
                while rclpy.ok():
                    try:
                        port = Serial(port_name, baud, timeout=5, write_timeout=10)
                        break
                    except SerialException as e:
                        error_msg = str(e)
                        logger.error("%s", e)
                        sleep(3)

                client = CaBotArduinoSerial(port, baud)
                delegate.owner = client
                client.delegate = delegate
                updater.setHardwareID(port_name)
                topic_alive = None
                client.start()

                rate = node.create_rate(2)
                while client.is_alive:
                    rate.sleep()
            except KeyboardInterrupt as e:
                logger.info("KeyboardInterrupt")
                rclpy.shutdown("user interrupted")
                break
            except SerialException as e:
                error_msg = str(e)
                logger.error(error_msg)
                sleep(sleep_time)
                continue
            except OSError as e:
                error_msg = str(e)
                logger.error(error_msg)
                traceback.print_exc(file=sys.stdout)
                sleep(sleep_time)
                continue
            except IOError as e:
                error_msg = str(e)
                logger.error(error_msg)
                logger.error("try to reconnect usb")
                sleep(sleep_time)
                continue
            except termios.error as e:
                error_msg = str(e)
                logger.error(error_msg)
                logger.error("connection disconnected")
                sleep(sleep_time)
                continue
            except SystemExit as e:
                break
            except:
                logger.error(F"{sys.exc_info()[0]}")
                traceback.print_exc(file=sys.stdout)
                rclpy.shutdown()
                sys.exit()

    import threading
    thread = threading.Thread(target=run, daemon=True)
    thread.start()

    try:
        rclpy.spin(node)
    except:
        pass
