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

from cabot.arduino_serial import CaBotArduinoSerialDelegate, CaBotArduinoSerial
import logging
import sys
import struct
import termios
import traceback
import time

from serial import Serial, SerialException

import rclpy
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.msg import ParameterDescriptor

from sensor_msgs.msg import Imu, FluidPressure, Temperature
from std_msgs.msg import Bool, UInt8, UInt8MultiArray, Int8, Int16, Float32, String
from std_srvs.srv import SetBool
from diagnostic_updater import Updater, DiagnosticTask, HeaderlessTopicDiagnostic, FrequencyStatusParam
from diagnostic_msgs.msg import DiagnosticStatus


# global variables
NUMBER_OF_BUTTONS = 5
topic_alive = None

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


def set_touch_speed_active_mode(msg):
    global touch_speed_active_mode

    touch_speed_active_mode = msg.data

    resp = SetBool.Response()
    if touch_speed_active_mode:
        resp.message = "touch speed active mode = True"
    else:
        resp.message = "touch speed active mode = False"
    resp.success = True
    return resp


class TopicCheckTask(HeaderlessTopicDiagnostic):
    def __init__(self, node, updater, name, freq):
        super().__init__(name, updater, FrequencyStatusParam({'min': freq, 'max': freq}, 0.1, 2))

    def tick(self):
        global topic_alive
        topic_alive = time.time()
        super().tick()


class CheckConnectionTask(DiagnosticTask):
    def __init__(self, name):
        super().__init__(name)

    def run(self, stat):
        global client, port, topic_alive
        if client is None:
            if error_msg is None:
                stat.summary(DiagnosticStatus.WARN, "connecting")
            else:
                stat.summary(DiagnosticStatus.ERROR, error_msg)
        else:
            if topic_alive and time.time() - topic_alive > 1:
                logger.error("connected but no message comming")
                stat.summary(DiagnosticStatus.ERROR, "connected but no message comming")
                client = None
                port = None
                topic_alive = None
            else:
                stat.summary(DiagnosticStatus.OK, "working")
        return stat


class ROSDelegate(CaBotArduinoSerialDelegate):
    def __init__(self):
        self.owner = None

        self.touch_raw_pub = node.create_publisher(Int16, "touch_raw", 10)
        self.touch_pub = node.create_publisher(Int16, "touch", 10)
        self.button_pub = node.create_publisher(Int8, "pushed", 10)
        self.btn_pubs = []
        for i in range(0, NUMBER_OF_BUTTONS):
            self.btn_pubs.append(node.create_publisher(Bool, "pushed_%d" % (i+1), 10))
        self.imu_pub = node.create_publisher(Imu, "imu", 10)
        self.imu_last_topic_time = None
        self.calibration_pub = node.create_publisher(UInt8MultiArray, "calibration", 10)
        self.pressure_pub = node.create_publisher(FluidPressure, "pressure", 10)
        self.temperature_pub = node.create_publisher(Temperature, "temperature", 10)
        self.wifi_pub = node.create_publisher(String, "wifi", 10)

        self.vib1_sub = node.create_subscription(UInt8, "vibrator1", self.vib_callback(0x20), 10)
        self.vib2_sub = node.create_subscription(UInt8, "vibrator2", self.vib_callback(0x21), 10)
        self.vib3_sub = node.create_subscription(UInt8, "vibrator3", self.vib_callback(0x22), 10)
        self.vib4_sub = node.create_subscription(UInt8, "vibrator4", self.vib_callback(0x23), 10)

    def vib_callback(self, cmd):
        def callback(msg):
            data = bytearray()
            data.append(msg.data)
            self.owner.send_command(cmd, data)
        return callback

    def system_time(self):
        now = node.get_clock().now().nanoseconds
        # logger.info("current time={}".format(node.get_clock().now()))
        return (int(now / 1000000000), int(now % 1000000000))

    def stopped(self):
        global client, port, topic_alive
        client = None
        port = None
        topic_alive = None
        logger.error("stopped")

    def log(self, level, text):
        if level == logging.INFO:
            logger.info(text)
        if level == logging.WARN:
            logger.warn(text)
        if level == logging.ERROR:
            logger.error(text)

    def log_throttle(self, level, interval, text):
        if level == logging.INFO:
            logger.info_throttle(interval, text)
        if level == logging.WARN:
            logger.warn_throttle(interval, text)
        if level == logging.ERROR:
            logger.error_throttle(interval, text)

    def get_param(self, name, callback):
        pd = ParameterDescriptor()
        val = None
        try:
            if name == "run_imu_calibration":
                pd.type = ParameterType.PARAMETER_BOOL
            elif name == "calibration_params":
                pd.type = ParameterType.PARAMETER_INTEGER_ARRAY
            elif name == "touch_params":
                pd.type = ParameterType.PARAMETER_INTEGER_ARRAY
            else:
                logger.info(F"Parameter {name} is not defined")
                callback([])

            if not node.has_parameter(name):
                val = node.declare_parameter(name, descriptor=pd).value
            else:
                try:
                    val = node.get_parameter(name).value
                except rclpy.exceptions.ParameterUninitializedException:
                    callback([])

        except:  # noqa #722
            logger.error(traceback.format_exc())
        finally:
            logger.info(F"get_param {name}={val}")
            if val is not None:
                callback(val)
                return
            callback([])

    def process_imu_data(self, data):
        # discard possible corrupted data
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
        imu_msg.header.stamp.sec = struct.unpack('i', struct.pack('f', data2[0]))[0]
        imu_msg.header.stamp.nanosec = struct.unpack('i', struct.pack('f', data2[1]))[0]
        if self.imu_last_topic_time is not None:
            if rclpy.time.Time.from_msg(self.imu_last_topic_time) > rclpy.time.Time.from_msg(imu_msg.header.stamp):
                logger.error("IMU timestamp is not consistent, drop a message\n"
                             "last imu time:{} > current imu time:{}".format(self.imu_last_topic_time, imu_msg.header.stamp),
                             throttle_duration_sec=1.0)
                return

        imu_msg.header.frame_id = "imu_frame"
        self.imu_last_topic_time = imu_msg.header.stamp
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
        return imu_msg

    def process_button_data(self, msg):
        for i in range(0, NUMBER_OF_BUTTONS):
            temp = Bool()
            temp.data = ((msg.data >> i) & 0x01) == 0x01
            self.btn_pubs[i].publish(temp)

    def publish(self, cmd, data):
        # logger.info("%x: %d", cmd, int.from_bytes(data, "little"))
        # logger.info("%x: %s", cmd, str(data));
        # self.log_throttle(logging.INFO, 1, "got data %x"%(cmd))
        if cmd == 0x10:  # touch
            msg = Int16()
            msg.data = int.from_bytes(data, 'little')
            self.touch_pub.publish(msg)
            touch_callback(msg)
            touch_check_task.tick()
        if cmd == 0x11:  # touch_raw
            msg = Int16()
            msg.data = int.from_bytes(data, 'little')
            self.touch_raw_pub.publish(msg)
        if cmd == 0x12:  # buttons
            msg = Int8()
            msg.data = int.from_bytes(data, 'little')
            self.button_pub.publish(msg)
            self.process_button_data(msg)
            button_check_task.tick()
        if cmd == 0x13:  # imu
            msg = self.process_imu_data(data)
            self.imu_pub.publish(msg)
            imu_check_task.tick()
        if cmd == 0x14:  # calibration
            msg = UInt8MultiArray()
            msg.data = data
            self.calibration_pub.publish(msg)
        if cmd == 0x15:  # pressure
            msg = FluidPressure()
            msg.fluid_pressure = struct.unpack('f', data)[0]
            msg.variance = 0.0
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.header.frame_id = "bmp_frame"
            self.pressure_pub.publish(msg)
            pressure_check_task.tick()
        if cmd == 0x16:  # temperature
            msg = Temperature()
            msg.temperature = struct.unpack('f', data)[0]
            msg.variance = 0.0
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.header.frame_id = "bmp_frame"
            self.temperature_pub.publish(msg)
            temp_check_task.tick()
        if cmd == 0x20:  # wifi
            msg = String()
            msg.data = data.decode()
            self.wifi_pub.publish(msg)


#def shutdown_hook(signal_num, frame):
#    print("shutdown_hook cabot_serial.py (cabot)")
#    sys.exit(0)
#
#
#signal.signal(signal.SIGINT, shutdown_hook)


if __name__ == "__main__":
    rclpy.init()
    node = rclpy.create_node("cabot_serial_node")
    logger = node.get_logger()
    logger.info("CABOT ROS Serial Python Node")

    # touch speed control
    node.declare_parameter('touch_speed_max', 2.0)
    node.declare_parameter('touch_speed_max_inactive', 0.5)
    node.declare_parameter('port', '/dev/ttyCABOT')
    node.declare_parameter('baud', 115200)

    touch_speed_max_speed = node.get_parameter('touch_speed_max').value
    touch_speed_max_speed_inactive = node.get_parameter('touch_speed_max_inactive').value
    touch_speed_switched_pub = node.create_publisher(Float32, "touch_speed_switched", 10)
    set_touch_speed_active_mode_srv = node.create_service(SetBool, "set_touch_speed_active_mode", set_touch_speed_active_mode)

    # Diagnostic Updater
    updater = Updater(node)
    imu_check_task = TopicCheckTask(node, updater, "IMU", 100)
    touch_check_task = TopicCheckTask(node, updater, "Touch Sensor", 50)
    button_check_task = TopicCheckTask(node, updater, "Push Button", 50)
    pressure_check_task = TopicCheckTask(node, updater, "Pressure", 2)
    temp_check_task = TopicCheckTask(node, updater, "Temperature", 2)
    updater.add(CheckConnectionTask("Serial Connection"))

    # add the following line into /etc/udev/rules.d/10-local.rules
    client = None
    port = None
    port_name = node.get_parameter('port').value
    port_names = [port_name]
    port_index = 0
    baud = node.get_parameter('baud').value

    sleep_time = 3
    error_msg = None
    delegate = ROSDelegate()

    timer = None

    def run_once():
        global client
        logger.debug("run_once", throttle_duration_sec=1.0)

        if client is None:
            return
        try:
            client.run_once()
        except SerialException as e:
            error_msg = str(e)
            logger.error(error_msg)
            client = None
            if timer:
                timer.cancel()
        except OSError as e:
            error_msg = str(e)
            logger.error(error_msg)
            traceback.print_exc(file=sys.stdout)
            client = None
            if timer:
                timer.cancel()
        except IOError as e:
            error_msg = str(e)
            logger.error(error_msg)
            logger.error("try to reconnect usb")
            client = None
            if timer:
                timer.cancel()
        except termios.error as e:
            error_msg = str(e)
            logger.error(error_msg)
            logger.error("connection disconnected")
            client = None
            if timer:
                timer.cancel()
        except SystemExit:
            pass
        except:  # noqa: E722
            logger.error(traceback.format_exc())
            rclpy.shutdown()
            sys.exit()

    def polling():
        global port, port_index, port_name, client, topic_alive, timer

        logger.debug(f"polling, {client}")

        if client and client.is_alive:
            return

        client = None
        port = None
        port_name = port_names[port_index]
        port_index = (port_index + 1) % len(port_names)
        logger.info("Connecting to %s at %d baud" % (port_name, baud))
        
        try:
            port = Serial(port_name, baud, timeout=5, write_timeout=10)
        except SerialException as e:
            logger.error(f"{e}")
            return

        client = CaBotArduinoSerial(port, baud)
        delegate.owner = client
        client.delegate = delegate
        updater.setHardwareID(port_name)
        topic_alive = None

        client.start()
        logger.info("serial is opened")
        timer = node.create_timer(0.001, run_once)

    temp = node.create_timer(1, polling)

    try:
        rclpy.spin(node)
    except:  # noqa: E722
        pass
