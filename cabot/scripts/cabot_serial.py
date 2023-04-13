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
import rclpy.clock
import rclpy.node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.msg import ParameterDescriptor

from sensor_msgs.msg import Imu, FluidPressure, Temperature
from std_msgs.msg import Bool, UInt8, UInt8MultiArray, Int8, Int16, Float32, String
from std_srvs.srv import SetBool
from diagnostic_updater import Updater, DiagnosticTask, HeaderlessTopicDiagnostic, FrequencyStatusParam
from diagnostic_msgs.msg import DiagnosticStatus


# global variables
client = None
port = None
timer = None
error_msg = None

NUMBER_OF_BUTTONS = 5
topic_alive = None


class TopicCheckTask(HeaderlessTopicDiagnostic):
    def __init__(self, node, updater, name, freq):
        super().__init__(name, updater, FrequencyStatusParam({'min': freq, 'max': freq}, 0.1, 2))

    def tick(self):
        global topic_alive
        topic_alive = time.time()
        super().tick()


class CheckConnectionTask(DiagnosticTask):
    def __init__(self, node, name):
        super().__init__(name)
        self.node = node

    def run(self, stat):
        global client, topic_alive, port
        if client is None:
            if error_msg is None:
                stat.summary(DiagnosticStatus.WARN, "connecting")
            else:
                stat.summary(DiagnosticStatus.ERROR, error_msg)
        else:
            if topic_alive and time.time() - topic_alive > 1:
                self.node.get_logger().error("connected but no message comming")
                stat.summary(DiagnosticStatus.ERROR, "connected but no message comming")
                client = None
                port = None
                topic_alive = None
            else:
                stat.summary(DiagnosticStatus.OK, "working")
        return stat


class CaBotSerialNode(rclpy.node.Node, CaBotArduinoSerialDelegate):
    def __init__(self):
        super().__init__("cabot_serial_node")
        self.client = None

        self.touch_raw_pub = self.create_publisher(Int16, "touch_raw", qos_profile_sensor_data)
        self.touch_pub = self.create_publisher(Int16, "touch", qos_profile_sensor_data)
        self.button_pub = self.create_publisher(Int8, "pushed", qos_profile_sensor_data)
        self.btn_pubs = []
        for i in range(0, NUMBER_OF_BUTTONS):
            self.btn_pubs.append(self.create_publisher(Bool, "pushed_%d" % (i+1), qos_profile_sensor_data))
        self.imu_pub = self.create_publisher(Imu, "imu", qos_profile_sensor_data)
        self.imu_last_topic_time = None
        self.calibration_pub = self.create_publisher(UInt8MultiArray, "calibration", 10)
        self.pressure_pub = self.create_publisher(FluidPressure, "pressure", 10)
        self.temperature_pub = self.create_publisher(Temperature, "temperature", 10)
        self.wifi_pub = self.create_publisher(String, "wifi", 10)

        self.vib1_sub = self.create_subscription(UInt8, "vibrator1", self.vib_callback(0x20), 10)
        self.vib2_sub = self.create_subscription(UInt8, "vibrator2", self.vib_callback(0x21), 10)
        self.vib3_sub = self.create_subscription(UInt8, "vibrator3", self.vib_callback(0x22), 10)
        self.vib4_sub = self.create_subscription(UInt8, "vibrator4", self.vib_callback(0x23), 10)

        # touch speed control
        # touch speed active mode
        # True:  Touch - go,    Not Touch - no go
        # False: Touch - no go, Not Touch - go
        self.touch_speed_active_mode = True
        self.touch_speed_max_speed = self.declare_parameter('touch_speed_max', 2.0).value
        self.touch_speed_max_speed_inactive = self.declare_parameter('touch_speed_max_inactive', 0.5).value
        transient_local_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.touch_speed_switched_pub = self.create_publisher(Float32, "touch_speed_switched", transient_local_qos)
        self.set_touch_speed_active_mode_srv = self.create_service(SetBool, "set_touch_speed_active_mode", self.set_touch_speed_active_mode)

        # Diagnostic Updater
        self.updater = Updater(self)
        self.imu_check_task = TopicCheckTask(self, self.updater, "IMU", 100)
        self.touch_check_task = TopicCheckTask(self, self.updater, "Touch Sensor", 50)
        self.button_check_task = TopicCheckTask(self, self.updater, "Push Button", 50)
        self.pressure_check_task = TopicCheckTask(self, self.updater, "Pressure", 2)
        self.temp_check_task = TopicCheckTask(self, self.updater, "Temperature", 2)
        self.updater.add(CheckConnectionTask(self, "Serial Connection"))

        self.client_logger = rclpy.logging.get_logger("arduino_serial")

    def vib_callback(self, cmd):
        def callback(msg):
            data = bytearray()
            data.append(msg.data)
            self.client.send_command(cmd, data)
        return callback

    def system_time(self):
        now = self.get_clock().now().nanoseconds
        # self.get_logger().info("current time={}".format(self.get_clock().now()))
        return (int(now / 1000000000), int(now % 1000000000))

    def stopped(self):
        global client, port, topic_alive
        client = None
        port = None
        topic_alive = None
        self.get_logger().error("stopped")

    def log(self, level, text):
        if level == logging.INFO:
            self.client_logger.info(text)
        if level == logging.WARN:
            self.client_logger.warn(text)
        if level == logging.ERROR:
            self.client_logger.error(text)
        if level == logging.DEBUG:
            self.client_logger.debug(text)

    def log_throttle(self, level, interval, text):
        if level == logging.INFO:
            self.client_logger.info_throttle(interval, text)
        if level == logging.WARN:
            self.client_logger.warn_throttle(interval, text)
        if level == logging.ERROR:
            self.client_logger.error_throttle(interval, text)
        if level == logging.DEBUG:
            self.client_logger.debug_throttle(interval, text)

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
                self.get_logger().info(F"Parameter {name} is not defined")
                callback([])

            if not self.has_parameter(name):
                val = self.declare_parameter(name, descriptor=pd).value
            else:
                try:
                    val = self.get_parameter(name).value
                except rclpy.exceptions.ParameterUninitializedException:
                    callback([])

        except:  # noqa #722
            self.get_logger().error(traceback.format_exc())
        finally:
            self.get_logger().info(F"get_param {name}={val}")
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
            return None

        imu_msg = Imu()
        imu_msg.orientation_covariance[0] = 0.1
        imu_msg.orientation_covariance[4] = 0.1
        imu_msg.orientation_covariance[8] = 0.1

        # convert float(32) to int(32)
        imu_msg.header.stamp.sec = struct.unpack('i', struct.pack('f', data2[0]))[0]
        imu_msg.header.stamp.nanosec = struct.unpack('i', struct.pack('f', data2[1]))[0]
        if self.imu_last_topic_time is not None:
            if rclpy.time.Time.from_msg(self.imu_last_topic_time) > rclpy.time.Time.from_msg(imu_msg.header.stamp):
                self.get_logger().error("IMU timestamp is not consistent, drop a message\n"
                                        "last imu time:{} > current imu time:{}".format(self.imu_last_topic_time, imu_msg.header.stamp),
                                        throttle_duration_sec=1.0)
                return None

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

    def touch_callback(self, msg):
        touch_speed_msg = Float32()
        if self.touch_speed_active_mode:
            touch_speed_msg.data = self.touch_speed_max_speed if msg.data else 0.0
            self.touch_speed_switched_pub.publish(touch_speed_msg)
        else:
            touch_speed_msg.data = 0.0 if msg.data else self.touch_speed_max_speed_inactive
            self.touch_speed_switched_pub.publish(touch_speed_msg)

    def set_touch_speed_active_mode(self, msg):
        self.touch_speed_active_mode = msg.data
        resp = SetBool.Response()
        if self.touch_speed_active_mode:
            resp.message = "touch speed active mode = True"
        else:
            resp.message = "touch speed active mode = False"
        resp.success = True
        return resp

    def publish(self, cmd, data):
        # self.get_logger().info("%x: %d", cmd, int.from_bytes(data, "little"))
        # self.get_logger().info("%x: %s", cmd, str(data));
        # self.log_throttle(logging.INFO, 1, "got data %x"%(cmd))
        if cmd == 0x10:  # touch
            msg = Int16()
            msg.data = int.from_bytes(data, 'little')
            self.touch_pub.publish(msg)
            self.touch_callback(msg)
            self.touch_check_task.tick()
        if cmd == 0x11:  # touch_raw
            msg = Int16()
            msg.data = int.from_bytes(data, 'little')
            self.touch_raw_pub.publish(msg)
        if cmd == 0x12:  # buttons
            msg = Int8()
            msg.data = int.from_bytes(data, 'little')
            self.button_pub.publish(msg)
            self.process_button_data(msg)
            self.button_check_task.tick()
        if cmd == 0x13:  # imu
            msg = self.process_imu_data(data)
            if msg:
                self.imu_pub.publish(msg)
                self.imu_check_task.tick()
        if cmd == 0x14:  # calibration
            msg = UInt8MultiArray()
            msg.data = data
            self.calibration_pub.publish(msg)
        if cmd == 0x15:  # pressure
            msg = FluidPressure()
            msg.fluid_pressure = struct.unpack('f', data)[0]
            msg.variance = 0.0
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "bmp_frame"
            self.pressure_pub.publish(msg)
            self.pressure_check_task.tick()
        if cmd == 0x16:  # temperature
            msg = Temperature()
            msg.temperature = struct.unpack('f', data)[0]
            msg.variance = 0.0
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "bmp_frame"
            self.temperature_pub.publish(msg)
            self.temp_check_task.tick()
        if cmd == 0x20:  # wifi
            msg = String()
            msg.data = data.decode()
            self.wifi_pub.publish(msg)


def main():
    global client, port, timer

    rclpy.init()
    node = CaBotSerialNode()
    logger = node.get_logger()
    logger.info("CABOT ROS Serial Python Node")
    port_name = node.declare_parameter('port', '/dev/ttyCABOT').value
    baud = node.declare_parameter('baud', 115200).value

    def run_once():
        global client, error_msg
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

    sys_clock = rclpy.clock.Clock()

    # polling to check if client (arduino) is disconnected and keep trying to reconnect
    def polling():
        global port, client, topic_alive, timer

        logger.debug(f"polling, {client}")
        if client and client.is_alive:
            return
        client = None
        port = None
        logger.info("Connecting to %s at %d baud" % (port_name, baud))
        try:
            port = Serial(port_name, baud, timeout=5, write_timeout=10)
        except SerialException as e:
            logger.error(f"{e}")
            return
        client = CaBotArduinoSerial(port, baud)
        node.client = client
        client.delegate = node
        node.updater.setHardwareID(port_name)
        topic_alive = None
        client.start()
        logger.info("Serial is ready")
        timer = node.create_timer(0.001, run_once, clock=sys_clock)

    node.create_timer(1, polling, clock=sys_clock)

    try:
        rclpy.spin(node)
    except:  # noqa: E722
        logger.debug(traceback.format_exc())
        pass


if __name__ == "__main__":
    main()
