#!/usr/bin/env python

 ###############################################################################
 # Copyright (c) 2019  Carnegie Mellon University
 #
 # Permission is hereby granted, free of charge, to any person obtaining a copy
 # of this software and associated documentation files (the "Software"), to deal
 # in the Software without restriction, including without limitation the rights
 # to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 # copies of the Software, and to permit persons to whom the Software is
 # furnished to do so, subject to the following conditions:
 #
 # The above copyright notice and this permission notice shall be included in
 # all copies or substantial portions of the Software.
 #
 # THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 # IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 # FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 # AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 # LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 # OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 # THE SOFTWARE.
 ###############################################################################

# Initial Committer
# EagleZ, dapengz@andrew.cmu.edu

'''ROS Imports'''
import sys
import rclpy
from rclpy.node import Node
from rclpy.exceptions import ROSInterruptException, InvalidServiceNameException
import logging
import traceback
from odriver_msgs.msg import MotorStatus
from odriver_msgs.msg import MotorTarget
from std_msgs.msg import Header

import threading

'''Functional Imports'''
#import serial
import odrive
from odrive.utils import dump_errors

from odrive.enums import *
import odrive.enums
axis_error_codes_tup =  [(name, value) for name, value in odrive.enums.__dict__.items() if "AXIS_ERROR_" in name]
motor_error_code_tup = [(name, value) for name, value in odrive.enums.__dict__.items() if "MOTOR_ERROR_" in name]
controller_error_code_tup = [(name, value) for name, value in odrive.enums.__dict__.items() if "CONTROLLER_ERROR_" in name]
encoder_error_code_tup = [(name, value) for name, value in odrive.enums.__dict__.items() if "ENCODER_ERROR_" in name]
sensorless_estimator_error_code_tup = [(name, value) for name, value in odrive.enums.__dict__.items() if "SENSORLESS_ESTIMATOR_ERROR_" in name]

import time
import numpy as np
import threading
from packaging import version

from diagnostic_updater import Updater, DiagnosticTask
from diagnostic_msgs.msg import DiagnosticStatus

from std_srvs.srv import SetBool

PRINTDEBUG=False

ODRIVE_VERSIONS=[[0,5,1],[0,5,4]]

'''Parameter'''
freq = 20 #Hz
pause_between_commands = 0.001 #sec
serialReading_timeout=0.01 #sec
serialWriting_timeout=0.01 #sec
lock=threading.Lock()
use_checksum=False

'''Configuarable parameter'''
meter_per_count = None
leftIs1 = False # left is axis0, right is axis1
signLeft = -1.0
signRight = 1.0
gainLeft = 1.0
gainRight = 1.0



'''Global Varaible'''
spd0_c, spd1_c = 0, 0
loop_ctrl_on = 0
odrv0 = None
odrv0_is_not_found = False
version_mismatched = False
use_index = False
index_not_found = False
first_motorTarget_arrive = False
new_motorTarget_arrive = False
node = None


def is_firmware_equal(odrv, od_version):
    return  (odrv.fw_version_major == od_version[0] and \
            odrv.fw_version_minor == od_version[1] and \
            odrv.fw_version_revision == od_version[2])


def is_firmware_supported(odrv):
    return any((is_firmware_equal(odrv,x) for x in ODRIVE_VERSIONS))

def clear_errors(odrv):
    fw_version = version.parse(".".join(map(str,[odrv.fw_version_major, odrv.fw_version_minor, odrv.fw_version_revision])))
    if version.parse("0.5.2") <= fw_version:
        odrv.clear_errors()
    else:
        odrv.axis0.clear_errors()
        odrv.axis1.clear_errors()


def find_controller(port, clear=False, reset_watchdog_error=False):
    '''Hardware Initialization'''
    global odrv0, odrv0_is_not_found, version_mismatched, use_index, index_not_found

    if clear:
        odrv0 = None

    odrv0_is_not_found = True
    while odrv0 is None and rclpy.ok:
        try:
            node.get_logger().info("Finding Odrive controller... : " + str(port))
            logging.basicConfig(level=logging.DEBUG)
            odrv0 = odrive.find_any(timeout=5) if not port else odrive.find_any(path=port, timeout=5)
        except:
            node.get_logger().error(traceback.format_exc())
            node.get_logger().error("Check Odrive connection: " + str(port) +  " doesn't exist! ")
            time.sleep(1)
            continue
        else:
            if odrv0 is None:
                return

    odrv0_is_not_found = False
    version_mismatched = False
    use_index = False
    index_not_found = False
    if not is_firmware_supported(odrv0):
        version_mismatched = True
        return
    if odrv0.axis0.encoder.config.use_index or odrv0.axis1.encoder.config.use_index:
        use_index = True
    if odrv0.axis0.encoder.index_found != 1 or odrv0.axis1.encoder.index_found != 1:
        index_not_found = True
    if use_index and index_not_found:
        return

    clear_errors(odrv0)

    print("Odrive connected as ", odrv0.serial_number) #odrv0.name, " !")
    node.get_logger().info("Odrive connected as " + str(odrv0.serial_number))

    od_setWatchdogTimer(0)
    # if an axis is stopped by watchdog timeout, reset the error status.
    if reset_watchdog_error:
        reset_error_watchdog_timer_expired()


def reset_error_watchdog_timer_expired():
    if odrv0.axis0.error & AXIS_ERROR_WATCHDOG_TIMER_EXPIRED != 0:#odrv0.axis0.error == AXIS_ERROR_WATCHDOG_TIMER_EXPIRED:
        odrv0.axis0.error = odrv0.axis0.error & ~AXIS_ERROR_WATCHDOG_TIMER_EXPIRED
        node.get_logger().info("Reset axis0.error from AXIS_ERROR_WATCHDOG_TIMER_EXPIRED to AXIS_ERROR_NONE.")
    if odrv0.axis1.error & AXIS_ERROR_WATCHDOG_TIMER_EXPIRED != 0:#odrv0.axis1.error == AXIS_ERROR_WATCHDOG_TIMER_EXPIRED:
        odrv0.axis1.error = odrv0.axis1.error & ~AXIS_ERROR_WATCHDOG_TIMER_EXPIRED
        node.get_logger().info("Reset axis1.error from AXIS_ERROR_WATCHDOG_TIMER_EXPIRED to AXIS_ERROR_NONE.")

def _axis_has_error(axis):
    return (axis.error != AXIS_ERROR_NONE
     or axis.motor.error != MOTOR_ERROR_NONE
     or axis.encoder.error != ENCODER_ERROR_NONE
     or axis.controller.error != CONTROLLER_ERROR_NONE
     or axis.sensorless_estimator.error != SENSORLESS_ESTIMATOR_ERROR_NONE)

def _odrv_has_error(odrv):
    return _axis_has_error(odrv.axis0) or _axis_has_error(odrv.axis1)

'''Subscriber Routine'''
def MotorTargetRoutine(data):
    global spd0_c
    global spd1_c
    global loop_ctrl_on
    global lock
    global first_motorTarget_arrive
    global new_motorTarget_arrive
    lock.acquire()
    loop_ctrl_on = data.loop_ctrl
    spd0_c = signLeft * gainLeft * data.spd_left / meter_per_round
    spd1_c = signRight * gainRight * data.spd_right / meter_per_round
    first_motorTarget_arrive = True
    new_motorTarget_arrive = True
    #print(spd0_c, spd1_c)
    if leftIs1:
        spd0_c, spd1_c = spd1_c, spd0_c
    lock.release()

class OdriveDeviceTask(DiagnosticTask):
    def __init__(self, name):
        super().__init__(name)

    def run(self, stat):
        try:
            if odrv0 is None:
                if odrv0_is_not_found:
                    stat.summary(DiagnosticStatus.ERROR, "could not find odrive")
                else:
                    stat.summary(DiagnosticStatus.WARN, "trying to connect to odrive")
                return stat

            if not is_firmware_supported(odrv0):
                stat.summary(DiagnosticStatus.ERROR,
                             "version %d.%d.%d is not matched with required version"%(
                                 odrv0.fw_version_major, odrv0.fw_version_minor, odrv0.fw_version_revision
                                 ))
                return stat

            if _odrv_has_error(odrv0):
                stat.summary(DiagnosticStatus.ERROR, dumps_errors(stat))
                return stat

            if odrv0.axis0.encoder.config.pre_calibrated != 1 or \
               odrv0.axis0.motor.config.pre_calibrated != 1 or \
               odrv0.axis1.encoder.config.pre_calibrated != 1 or \
               odrv0.axis1.motor.config.pre_calibrated != 1:
                stat.summary(DiagnosticStatus.ERROR, "Motor is not calibrated.")
                return stat


            if (odrv0.axis0.encoder.config.use_index and odrv0.axis0.encoder.index_found == False) or \
               (odrv0.axis1.encoder.config.use_index and odrv0.axis1.encoder.index_found == False):
                stat.summary(DiagnosticStatus.ERROR, "Encoder did not found z-index. Please turn the wheels a few times.")
                return stat

            stat.summary(DiagnosticStatus.OK,
                         "version: %d.%d.%d"%(odrv0.fw_version_major, odrv0.fw_version_minor, odrv0.fw_version_revision))
        except:
            pass
        return stat

def dumps_errors(stat):
    axes = [(name, getattr(odrv0, name)) for name in dir(odrv0) if name.startswith('axis')]
    axes.sort()
    anyError=False
    for name, axis in axes:
        axis_name = name
        # Flatten axis and submodules
        # (name, remote_obj, errorcode)
        module_decode_map = [
            ('axis', axis, {k: v for k, v in odrive.enums.__dict__ .items() if k.startswith("AXIS_ERROR_")}),
            ('motor', axis.motor, {k: v for k, v in odrive.enums.__dict__ .items() if k.startswith("MOTOR_ERROR_")}),
            ('fet_thermistor', axis.fet_thermistor, {k: v for k, v in odrive.enums.__dict__ .items() if k.startswith("THERMISTOR_CURRENT_LIMITER_ERROR")}),
            ('motor_thermistor', axis.motor_thermistor, {k: v for k, v in odrive.enums.__dict__ .items() if k.startswith("THERMISTOR_CURRENT_LIMITER_ERROR")}),
            ('encoder', axis.encoder, {k: v for k, v in odrive.enums.__dict__ .items() if k.startswith("ENCODER_ERROR_")}),
            ('controller', axis.controller, {k: v for k, v in odrive.enums.__dict__ .items() if k.startswith("CONTROLLER_ERROR_")}),
        ]

        # Module error decode
        for name, remote_obj, errorcodes in module_decode_map:
            if (remote_obj.error != 0):
                anyError = True
                foundError = False
                errors=[]
                errorcodes_tup = [(name, val) for name, val in errorcodes.items() if 'ERROR_' in name]
                for codename, codeval in errorcodes_tup:
                    if remote_obj.error & codeval != 0:
                        foundError = True
                        errors.append(codename)
                if not foundError:
                    errors.append('UNKNOWN ERROR')
                stat.add(axis_name+"_"+name, ",".join(errors))
            else:
                stat.add(axis_name+"_"+name, "no error")
    return "OK" if not anyError else "Error"

class TopicCheckTask(DiagnosticTask):
    def __init__(self, name, topic, topic_type, callback=lambda x:x):
        DiagnosticTask.__init__(self, name)
        print(topic_type)
        print(topic)
        self.sub = node.create_subscription(topic_type, topic, self.topic_callback, 10)
        self.callback = callback
        self.topic_count = 0

    def topic_callback(self, msg):
        self.callback(msg)
        self.topic_count += 1

    def run(self, stat):
        now = node.get_clock().now()

        if self.topic_count == 0:
            stat.summary(DiagnosticStatus.ERROR, "not working")
        else:
            stat.summary(DiagnosticStatus.OK, "working")
        self.topic_count = 0
        return stat


def _relaunch_odrive():
    node.get_logger().info('re-launching odrive..')
    set_odrive_power_proxy = None
    cli = node.create_client(SetBool, '/ace_battery_control/set_odrive_power')
    try:    
        cli.wait_for_service(timeout_sec=2.0)
        req = SetBool.Request()
        req.data = False
        cli.call_async(req)
        time.sleep(2.0)
        #set_odrive_power_proxy(True)
    except InvalidServiceNameException as se:
        node.get_logger().warn("_relaunch_odrive: Service call failed: %s"%se)
    except ROSInterruptException as re:
        node.get_logger().warn("_relaunch_odrive: wait_for_service failed: %s"%re)
    finally:
        # odrive would better be powered in any event.
        if set_odrive_power_proxy is not None:
            set_odrive_power_proxy(True)
            node.get_logger().info('re-launch odrive done')

def _need_relaunch_error_motor(axis):
    return (axis.motor.error & MOTOR_ERROR_CONTROL_DEADLINE_MISSED) != 0

def _need_relaunch_error(odrv):
    return _need_relaunch_error_motor(odrv.axis0) or _need_relaunch_error_motor(odrv.axis1)

def _error_recovery(relaunch = True):
    if _need_relaunch_error(odrv0):
        _relaunch_odrive()
    else:
        clear_errors(odrv0)
        if _odrv_has_error(odrv0) and relaunch:
            _relaunch_odrive()
        


'''Main()'''
def main():
    global node
    rclpy.init()
    node = Node("odrive_node")
    
    def spin():
        rclpy.spin(node)

    spin_thread = threading.Thread(target=spin, daemon=True)
    spin_thread.start()

    ## Diagnostic Updater
    updater = Updater(node)
    updater.add(TopicCheckTask("Motor Target", "motorTarget", MotorTarget, MotorTargetRoutine))
    updater.add(OdriveDeviceTask("ODrive"))

    pub = node.create_publisher(MotorStatus, 'motorStatus', 10)
#   sub = rclpy.create_subscription('motorTarget', MotorTarget, MotorTargetRoutine, 10)

    global meter_per_count, meter_per_round, leftIs1, signLeft, signRight, gainLeft, gainRight
    global new_motorTarget_arrive
    wheel_diameter = node.declare_parameter("wheel_diameter", 0.1).value
    count_per_round = node.declare_parameter("count_per_round", 100).value
    meter_per_count = wheel_diameter * np.pi / count_per_round
    meter_per_round = wheel_diameter * np.pi
    if node.has_parameter("left_is_1"): leftIs1 = node.declare_parameter("left_is_1", False).value
    if node.has_parameter("gain_left"): gainLeft = node.declare_parameter("gain_left", 1.0).value
    if node.has_parameter("gain_right"): gainRight = node.declare_parameter("gain_right", 1.0).value

    vel_gain = node.declare_parameter("vel_gain", 1.0).value
    vel_integrator_gain = node.declare_parameter("vel_integrator_gain", 10.0).value

    encoder_bandwidth = node.declare_parameter("encoder_bandwidth", 200).value
    motor_bandwidth = node.declare_parameter("motor_bandwidth", 200).value

    wtimer = node.declare_parameter("wd_timeout", 1.0).value

    wait_first_command = node.declare_parameter("wait_first_command", True).value # does not set watchdog timer before receiving first motorTarget input.
    reset_watchdog_error = node.declare_parameter("reset_watchdog", True).value # reset watchdog timeout error at start-up.
    connection_timeout = node.declare_parameter("connection_timeout", 5.0).value

    path = node.declare_parameter("path", '').value #specify path(e.g. usb:0001:0008) from .launch file, but not yet tested _aksg
    find_controller(path, reset_watchdog_error=reset_watchdog_error)

    # fuction to convert errorcode to a list of error name
    def errorcode_to_list(error_code, tup):
        error_list = []
        for codename, codeval in tup:
            if error_code & codeval != 0:
                error_list.append(codename)
        return error_list
    
    def errorcode_to_list_axis(error_code):
        return errorcode_to_list(error_code, axis_error_codes_tup)

    def errorcode_to_list_motor(error_code):
        return errorcode_to_list(error_code, motor_error_code_tup)

    def errorcode_to_list_controller(error_code):
        return errorcode_to_list(error_code, controller_error_code_tup)

    def errorcode_to_list_encoder(error_code):
        return errorcode_to_list(error_code, encoder_error_code_tup)

    def errorcode_to_list_sensorless_estimator(error_code):
        return errorcode_to_list(error_code, sensorless_estimator_error_code_tup)

    # adjust motor configuration
    def set_config():
        try:
            odrv0.axis0.controller.config.vel_gain = vel_gain
            odrv0.axis1.controller.config.vel_gain = vel_gain
            odrv0.axis0.controller.config.vel_integrator_gain = vel_integrator_gain
            odrv0.axis1.controller.config.vel_integrator_gain = vel_integrator_gain
            odrv0.axis0.encoder.config.bandwidth = encoder_bandwidth
            odrv0.axis1.encoder.config.bandwidth = encoder_bandwidth
            odrv0.axis0.motor.config.current_control_bandwidth = motor_bandwidth
            odrv0.axis1.motor.config.current_control_bandwidth = motor_bandwidth
        except:
            pass
    set_config()

    last_feed = 0

    rate = node.create_rate(freq)
    ms = MotorStatus()

    mode_written=None
    spd0_c_written,spd1_c_written=None,None

    class ShutDownHook:
        def stop_control(self):
            od_writeSpd(0, 0)
            od_writeSpd(1, 0)
            od_setWatchdogTimer(0)
            od_writeMode(0)

        def callback(self):
            self.stop_control()

    shutdown_hook = ShutDownHook()
    rclpy.get_default_context().on_shutdown(shutdown_hook.callback)

    # variables to manage connection error
    odrv0_is_active = True
    time_disconnect = node.get_clock().now()

    while rclpy.ok:
        # retry connection after timeout
        if not odrv0_is_active:
            diff_time = node.get_clock().now() - time_disconnect
            if diff_time.nanoseconds/1000000000 > connection_timeout:
                node.get_logger().warn("Odrive connection timeout. Retry finding odrive contoller...")
                time_disconnect = node.get_clock().now()
                find_controller(path, clear=True, reset_watchdog_error=reset_watchdog_error)
                set_config()

        if version_mismatched or (use_index and index_not_found):
            if odrv0_is_active:
                time_disconnect = node.get_clock().now()
            continue

        # check odrv0 remote object
        try:
            odrv0.axis0
            odrv0.axis1
        except:
            # if changes from True to False
            if odrv0_is_active:
                time_disconnect = node.get_clock().now()

            odrv0_is_active = False

            # reset written values
            mode_written=None
            spd0_c_written,spd1_c_written=None,None

            import traceback
            node.get_logger().error("Failed to access odrv0 axes.", throttle_duration_sec=5)
            rate.sleep()
            continue
        else:
            # recovery from inactive
            if not odrv0_is_active:
                try:
                    # reset odrv0 control
                    stop_control()
                    if reset_watchdog_error:
                        reset_error_watchdog_timer_expired()
                except:
                    import traceback
                    node.get_logger().error("Failed to reset odrv0 control.")
                    node.get_logger().error(traceback.format_exc())
                    rate.sleep()
                    continue
                else:
                    odrv0_is_active = True

        ## read error
        #getResponse("r axis0.motor.error", "error    0")
        #getResponse("r axis1.motor.error", "error    1")

        # error check
        try:
            if _odrv_has_error(odrv0):
                node.get_logger().error("odrv0.axis0.error=" +
                             str(errorcode_to_list_axis(odrv0.axis0.error)) +
                             ", odrv0.axis0.motor.error=" + 
                             str(errorcode_to_list_motor(odrv0.axis0.motor.error)) +
                             ", odrv0.axis0.controller.error=" + 
                             str(errorcode_to_list_controller(odrv0.axis0.controller.error)) +
                             ", odrv0.axis0.encoder.error=" + 
                             str(errorcode_to_list_encoder(odrv0.axis0.encoder.error)) +
                             ", odrv0.axis0.sensorless_estimator.error=" + 
                             str(errorcode_to_list_sensorless_estimator(odrv0.axis0.sensorless_estimator.error)) +
                             ", odrv0.axis1.error=" +                            
                             str(errorcode_to_list_axis(odrv0.axis1.error)) + 
                             ", odrv0.axis1.motor.error=" + 
                             str(errorcode_to_list_motor(odrv0.axis1.motor.error)) +
                             ", odrv0.axis1.controller.error=" + 
                             str(errorcode_to_list_controller(odrv0.axis1.controller.error)) +
                             ", odrv0.axis1.encoder.error=" + 
                             str(errorcode_to_list_encoder(odrv0.axis1.encoder.error)) +
                             ", odrv0.axis1.sensorless_estimator.error=" + 
                             str(errorcode_to_list_sensorless_estimator(odrv0.axis1.sensorless_estimator.error)),
                             throttle_duration_sec=5)
                node.get_logger().warn("Odrive error. trying recovery" + ("(relaunch)..." if odrv0_is_active else "..."))
                _error_recovery(relaunch = odrv0_is_active)
                time_disconnect = node.get_clock().now()
                odrv0_is_active = False
                rate.sleep()
                continue
        except:
            import traceback
            exception_string = traceback.format_exc()
            node.get_logger().error(exception_string)
            rate.sleep()
            continue

        # send new velocity command
        global lock
        lock.acquire()
        try:
            if(mode_written!=loop_ctrl_on):
                if PRINTDEBUG: print('w m ', loop_ctrl_on)
                if od_writeMode(loop_ctrl_on):
                    mode_written=loop_ctrl_on

            if(spd0_c_written!=spd0_c):
                if PRINTDEBUG: print('w 0 {:0.2f}'.format(spd0_c))
                if od_writeSpd(0,spd0_c):
                    spd0_c_written=spd0_c

            if(spd1_c_written!=spd1_c):
                if PRINTDEBUG: print('w 1 {:0.2f}'.format(spd1_c))
                if od_writeSpd(1,spd1_c):
                    spd1_c_written=spd1_c
            lock.release()
        except:
            lock.release()
            import traceback
            exception_string = traceback.format_exc()
            node.get_logger().error("Failed to set requested_state and vel_setpoint")
            node.get_logger().error(exception_string)
            rate.sleep()
            continue

        # set watchdog timer
        try:
            # enable watchdog timer after receiving at least one motorTarget
            if wait_first_command:
                if first_motorTarget_arrive:
                    od_setWatchdogTimer(wtimer)
            else:
                od_setWatchdogTimer(wtimer)

            # feed watchdog timer
            if new_motorTarget_arrive:
                # call watchdog_feed only when motorTarget is being updated. odrive motors stop when motorTarget update stops.
                od_feedWatchdogTimer()
                new_motorTarget_arrive = True
        except:
            import traceback
            exception_string = traceback.format_exc()
            node.get_logger().error("Failed to set watchdog timer")
            node.get_logger().error(exception_string)
            rate.sleep()
            continue

#        if 0 == wtimer:
#            wtimer = node.declare_parameter("wd_timeout", 1.0)
#            od_setWatchdogTimer(wtimer)
#        else:
#            nw = time.time()
#            if 0 == last_feed:
#                last_feed = nw
#            if nw - last_feed > wtimer * 0.8:
#                last_feed = nw
#                od_feedWatchdogTimer()


        enc0 = enc1 = spd0 = spd1 = None
        current_setpoint_0 = current_setpoint_1 = None
        current_measured_0 = current_measured_1 = None
        # update encoder counts and speed
        try:
            enc0, spd0 = odrv0.axis0.encoder.pos_estimate, odrv0.axis0.encoder.vel_estimate#getFloats(getResponse("f 0"))
            enc1, spd1 = odrv0.axis1.encoder.pos_estimate, odrv0.axis1.encoder.vel_estimate#getFloats(getResponse("f 1"))
            current_setpoint_0 = odrv0.axis0.motor.current_control.Iq_setpoint
            current_setpoint_1 = odrv0.axis1.motor.current_control.Iq_setpoint
            current_measured_0 = odrv0.axis0.motor.current_control.Iq_measured
            current_measured_1 = odrv0.axis1.motor.current_control.Iq_measured
        except:
            time.sleep(0.001)
            print("Reading TRY failed!")
            rate.sleep()
            import traceback
            exception_string = traceback.format_exc()
            node.get_logger().error(exception_string)
            continue
        
        if enc0 is not None and enc1 is not None and \
           spd0 is not None and spd1 is not None:
            ms.header.stamp = node.get_clock().now().to_msg()

            if leftIs1:
                ms.dist_left_c  = enc1
                ms.dist_right_c = enc0
                ms.spd_left_c   = spd1
                ms.spd_right_c  = spd0

                ms.current_setpoint_left = current_setpoint_1 * signLeft
                ms.current_setpoint_right = current_setpoint_0 * signRight
                ms.current_measured_left = current_measured_1 * signLeft
                ms.current_measured_right = current_measured_0 * signRight
            else:
                ms.dist_left_c  = enc0
                ms.dist_right_c = enc1
                ms.spd_left_c   = spd0
                ms.spd_right_c  = spd1

                ms.current_setpoint_left = current_setpoint_0 * signLeft
                ms.current_setpoint_right = current_setpoint_1 * signRight
                ms.current_measured_left = current_measured_0 * signLeft
                ms.current_measured_right = current_measured_1 * signRight

            ms.dist_left_c  *= signLeft / gainLeft
            ms.dist_right_c *= signRight / gainRight
            ms.spd_left_c  *= signLeft / gainLeft
            ms.spd_right_c *= signRight / gainRight

            ms.dist_left  = ms.dist_left_c  * meter_per_round
            ms.dist_right = ms.dist_right_c * meter_per_round
            ms.spd_left  = ms.spd_left_c  * meter_per_round
            ms.spd_right = ms.spd_right_c * meter_per_round
            pub.publish(ms)

        rate.sleep()

    # stop motor before exit
    od_writeSpd(0,0)
    od_writeSpd(1,0)
    od_setWatchdogTimer(0)
    od_writeMode(0)
    #print "shutdown and reboot"
    #od_reboot()

def getFloats(text):
    if text is None:
        return None, None
    items = text.split(" ")
    if len(items) < 2:
        print("Illegal input:'%s'"%(text))
        return None, None
    try:
        return float(items[0]), float(items[1])
    except:
        print("Illegal input:'%s'"%(text))
        return None, None

def od_reboot():
    #od.write('sb\n')##assuming "sr\n" instead here.. https://docs.odriverobotics.com/ascii-protocol.html
    odrv0.reboot()

def od_writeSpd(ch, spd):
    try:
        ctrl = None
        if(ch==0):
            ctrl = odrv0.axis0.controller
        else:
            ctrl = odrv0.axis1.controller
        ctrl.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
        ctrl.input_vel = spd
        return 1
    except:
        raise

def od_setWatchdogTimer(sec):
    if 0 < sec:
        # store previous watchdog_timeout values to use them later
        prev_watchdog_timeout_0 = odrv0.axis0.config.watchdog_timeout
        prev_watchdog_timeout_1 = odrv0.axis1.config.watchdog_timeout

        odrv0.axis0.config.watchdog_timeout = sec
        odrv0.axis1.config.watchdog_timeout = sec

        # if previous watchdog_timeout == 0, reset watchdog timer by watchdog_feed to prevent immediate timeout.
        # watchdog_feed must be called after watchdog_timeout is set because watchdog_timeout seems to be used in watchdog_feed function.
        if prev_watchdog_timeout_0 == 0:
            odrv0.axis0.watchdog_feed()
        if prev_watchdog_timeout_1 == 0:
            odrv0.axis1.watchdog_feed()

        odrv0.axis0.config.enable_watchdog = True
        odrv0.axis1.config.enable_watchdog = True
    else:
        # disable watchdog timer before setting watchdog_timeout to 0
        odrv0.axis0.config.enable_watchdog = False
        odrv0.axis1.config.enable_watchdog = False
        odrv0.axis0.config.watchdog_timeout = sec
        odrv0.axis1.config.watchdog_timeout = sec

def od_feedWatchdogTimer():
    odrv0.axis0.watchdog_feed()
    odrv0.axis1.watchdog_feed()

def od_writeSpd_obsolete(ch,spd):
    spd_s=str(spd)[:7]
    ch=int(ch!=0)
    cmd='v ' +str(ch)+' '+spd_s
    result = od_write(cmd)
    if PRINTDEBUG: print( cmd, result)
    return result

def od_writeMode(loop_ctrl_on):
    try:
        if (loop_ctrl_on==1):
            odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
            odrv0.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
            return 1
        else:
            odrv0.axis0.requested_state = AXIS_STATE_IDLE
            odrv0.axis1.requested_state = AXIS_STATE_IDLE
            return 1
    except:
        raise

# For more information about the number in command:
# https://github.com/madcowswe/ODrive/blob/master/tools/odrive/enums.py OR contact EagleZ
def od_writeMode_obsolete(loop_ctrl_on):
    if (loop_ctrl_on==1):
        result = (od_write('w axis0.requested_state 8') and od_write('w axis1.requested_state 8'))
    else:
        result = (od_write('w axis0.requested_state 1') and od_write('w axis1.requested_state 1'))
    return result

def checksum(st):
    cs = 0
    for c in st:
        cs = cs ^ ord(c)
    return cs & 0xff

def od_write_obsolete(st): # To be improved
    if use_checksum:
        cmd = "{}*{}\n".format(st, checksum(st))
    else:
        cmd = "{}\n".format(st)
    length = len(cmd)
    try:
        if PRINTDEBUG: print (cmd.rstrip())
        if (odrv0.write(cmd)==length):
            return 1
    except:
        print ("FAILED!!!! "+str(cmd))
    return 0

def getResponse(st, comment=None): # To be improved
    result = od_write(st)

    time.sleep(pause_between_commands)
    line=odrv0.readline().rstrip()
    if use_checksum:
        if not "*" in line:
            print ("Response without checksum '{}'".format(line))
            return None

        print (line)
        res, cs = line.split("*")
        try:
            if checksum(res) != int(cs):
                print ("checksum is different {} != {}".format(checksum(res), int(cs)))
                return None
        except:
            print ("checksum is not int")
            return None
    else:
        if PRINTDEBUG: print (line)
        res = line

    if comment:
        print ("{}: {:20s} - written[{}]: {}, ".format(comment, res, result, st))
    return res


'''Run'''
if __name__ == '__main__':
    main()
