#!/usr/bin/env python3

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


'''ROS Imports'''
from odrive.pyfibre import fibre
import signal
import sys
import rclpy
from rclpy.duration import Duration
from rclpy.exceptions import ROSInterruptException, InvalidServiceNameException
import time
import logging
import traceback
from odriver_msgs.msg import MotorStatus
from odriver_msgs.msg import MotorTarget
from std_msgs.msg import Header


#import serial
import odrive
from odrive.utils import dump_errors, format_errors

from odrive.enums import ODriveError, AxisState, ControlMode, ProcedureResult, ComponentStatus
import odrive.enums
odrive_error_codes_tup = [(name, value) for name, value in odrive.enums.__dict__.items() if "ODRIVE_ERROR_" in name]
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

ODRIVE_VERSIONS=[[0,6,5],[0,6,6]]

'''Parameter'''
freq = 40 #Hz
pause_between_commands = 0.001 #sec
serialReading_timeout=0.01 #sec
serialWriting_timeout=0.01 #sec
lock=threading.Lock()
use_checksum=False

'''Configuarable parameter'''
meter_per_count = None
leftIs1 = False # left is axis0, right is axis1
isClockwise = True # set true if sign = 1 corresponds to the clockwise direction. set false if sign = 1 corresponds to the counter-clockwise direction.
signLeft = -1.0
signRight = 1.0
gainLeft = 1.0
gainRight = 1.0


'''Global Varaible'''
spd0_c, spd1_c = 0, 0
loopCtrl_on = 0
odrvs = [None]*2
odrv_is_not_found = [False, False]
version_mismatched = False
use_index = False
index_not_found = False
count_motorTarget = None
previous_count_motorTarget = None
fw_version_str = ""
fw_version = None


def is_firmware_equal(odrv, od_version):
    return  (odrv.fw_version_major == od_version[0] and \
            odrv.fw_version_minor == od_version[1] and \
            odrv.fw_version_revision == od_version[2])


def is_firmware_supported(odrv):
    return any((is_firmware_equal(odrv,x) for x in ODRIVE_VERSIONS))

def clear_errors(odrv):
    global fw_version
    
    if version.parse("0.5.2") <= fw_version:
        odrv.clear_errors()
    else: # fw_version <= 0.5.1
        # The following try block throws an error when an odrv object returns a wrong version number due to a bug related to firmware.
        try:
            odrvs[0].axis0.clear_errors()
            odrvs[1].axis0.clear_errors()
        except AttributeError:
            odrv.clear_errors()


node = None
logger = None



def find_controller(odrv_index, odrv_serial_number, clear=False, reset_watchdog_error=False):
    '''Hardware Initialization'''
    global odrvs, odrv_is_not_found, version_mismatched, fw_version, channel_termination_token

    # channel_termination_token = fibre.Event()
    # logger.info("channel_termination_token : " + str(channel_termination_token))
    if clear:
        odrvs[odrv_index] = None

    odrv_is_not_found[odrv_index] = True
    while odrvs[odrv_index] is None and rclpy.ok:
        try:
            logger.info("Finding Odrive controller... (Serial Number: " + str(odrv_serial_number)+")")
            logging.basicConfig(level=logging.DEBUG)
            odrvs[odrv_index] = odrive.find_any(timeout=5, serial_number=odrv_serial_number)
        except:
            logger.error(traceback.format_exc())
            logger.error("Check Odrive connection: " + str(odrv_serial_number)+ " (Serial Number) doesn't exist! ")
            time.sleep(1)
            continue
        else:
            if odrvs[odrv_index] is None:
                return
    
    odrv_is_not_found[odrv_index] = False
    version_mismatched = False
    use_index = False
    index_not_found = False

    if not is_firmware_supported(odrvs[odrv_index]):
        logger.error("Odriver version is mismatched!")
        version_mismatched = True
        return
    
    if odrvs[odrv_index].axis0.commutation_mapper.config.use_index_gpio:
        use_index = True
    if odrvs[odrv_index].axis0.commutation_mapper.config.index_gpio != 10:
        index_not_found = True
    if use_index and index_not_found:
        return
    
    if fw_version_str != "":
        fw_version = version.parse(fw_version_str)
    else:
        fw_version = version.parse(".".join(map(str,[odrvs[odrv_index].fw_version_major, odrvs[odrv_index].fw_version_minor, odrvs[odrv_index].fw_version_revision])))

    clear_errors(odrvs[odrv_index])
    logger.info("Connected to Odrive S1 " + str(odrv_serial_number) + " as odrv" + str(odrv_index))    

    od_setWatchdogTimer(0, odrv_index)
    # if an axis is stopped by watchdog timeout, reset the error status.
    if reset_watchdog_error:
        reset_error_watchdog_timer_expired(odrv_index)


def reset_error_watchdog_timer_expired(odrv_index):
    if odrvs[odrv_index].axis0.active_errors & ODriveError.WATCHDOG_TIMER_EXPIRED != 0:
        odrvs[odrv_index].axis0.active_errors= odrvs[odrv_index].axis0.active_errors & ~ODriveError.WATCHDOG_TIMER_EXPIRED
        logger.info("Reset odrv"+str(odrv_index)+".axis0.error from AXIS_ERROR_WATCHDOG_TIMER_EXPIRED to AXIS_ERROR_NONE.")


def _odrv_has_error(odrv):
    return (odrv.axis0.active_errors != ODriveError.NONE
      or odrv.axis0.disarm_reason != ODriveError.NONE)
    #  or odrv.spi_encoder0.status != ComponentStatus.NOMINAL)

'''Subscriber Routine'''
def MotorTargetRoutine(data):
    global spd0_c
    global spd1_c
    global loopCtrl_on
    global lock
    global count_motorTarget
    lock.acquire()
    loopCtrl_on = data.loop_ctrl
    spd0_c = signLeft * gainLeft * data.spd_left / meter_per_round
    spd1_c = signRight * gainRight * data.spd_right / meter_per_round
    if count_motorTarget is None:
        count_motorTarget = 1
    else:
        count_motorTarget += 1
    if leftIs1:
        spd0_c, spd1_c = spd1_c, spd0_c
    lock.release()

class OdriveDeviceTask(DiagnosticTask):
    def __init__(self, name):
        super().__init__(name)

    def run(self, stat):
        try:
            global lock
            lock.acquire()
            if (odrvs[0] is None) or (odrvs[1] is None):
                if odrv_is_not_found[0] or odrv_is_not_found[1]:
                    stat.summary(DiagnosticStatus.ERROR, "could not find odrive")
                else:
                    stat.summary(DiagnosticStatus.WARN, "trying to connect to odrive")
                lock.release()
                return stat

            if not is_firmware_supported(odrvs[0]):
                stat.summary(DiagnosticStatus.ERROR,
                             "version %d.%d.%d is not matched with required version"%(
                                 odrvs[0].fw_version_major, odrvs[0].fw_version_minor, odrvs[0].fw_version_revision
                                 ))
                lock.release()
                return stat

            if _odrv_has_error(odrvs[0]):
                stat.summary(DiagnosticStatus.ERROR, str(format_errors(odrvs[0])))
                lock.release()
                return stat

            if _odrv_has_error(odrvs[1]):
                stat.summary(DiagnosticStatus.ERROR, str(format_errors(odrvs[1])))
                lock.release()
                return stat

            if not odrvs[0].axis0.config.motor.phase_inductance_valid or \
               not odrvs[1].axis0.config.motor.phase_inductance_valid:
                stat.summary(DiagnosticStatus.ERROR, "Motor is not calibrated.")
                lock.release()
                return stat
            if (odrvs[0].axis0.commutation_mapper.config.use_index_gpio and odrvs[0].axis0.commutation_mapper.config.index_gpio != 10) or \
               (odrvs[1].axis0.commutation_mapper.config.use_index_gpio and odrvs[1].axis0.commutation_mapper.config.index_gpio != 10):
                stat.summary(DiagnosticStatus.ERROR, "Encoder did not found z-index. Please turn the wheels a few times.")
                lock.release()
                return stat

            stat.summary(DiagnosticStatus.OK,
                         "version: %d.%d.%d"%(odrvs[0].fw_version_major, odrvs[0].fw_version_minor, odrvs[0].fw_version_revision))
            
            lock.release()
        except:
            pass
        return stat


class TopicCheckTask(DiagnosticTask):
    def __init__(self, name, topic, topic_type, callback=lambda x:x):
        DiagnosticTask.__init__(self, name)
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
    logger.info('re-launching odrive..')
    cli = node.create_client(SetBool, '/ace_battery_control/set_odrive_power')
    if cli.wait_for_service(timeout_sec=2.0):
        req = SetBool.Request()
        # turn off
        req.data = False
        cli.call(req)
        # wait 2 secs and turn on
        time.sleep(2.0)
        req.data = True
        cli.call(req)


def _need_relaunch_error_motor(axis):
    return (axis.motor.error & MOTOR_ERROR_CONTROL_DEADLINE_MISSED) != 0

def _need_relaunch_error(odrv):
    return _need_relaunch_error_motor(odrv.axis0)

def _error_recovery(relaunch = True):
    if _need_relaunch_error(odrvs[0]) or _need_relaunch_error(odrvs[1]):
        _relaunch_odrive()
    else:
        clear_errors(odrvs[0])
        clear_errors(odrvs[1])
        if (_odrv_has_error(odrvs[0]) and relaunch) or (_odrv_has_error(odrvs[1]) and relaunch) :
            _relaunch_odrive()


'''Main()'''
def main():
    pub = node.create_publisher(MotorStatus, 'motorStatus', 10)
    rate = node.create_rate(freq)

    global meter_per_count, meter_per_round, leftIs1, isClockwise, signLeft, signRight, gainLeft, gainRight, fw_version_str, fw_version
    global count_motorTarget, previous_count_motorTarget
    wheel_diameter = node.declare_parameter("wheel_diameter", 0.037).value
    count_per_round = node.declare_parameter("count_per_round", 8096).value
    meter_per_count = wheel_diameter * np.pi / count_per_round
    meter_per_round = wheel_diameter * np.pi

    leftIs1 = node.declare_parameter("left_is_1", leftIs1).value
    isClockwise = node.declare_parameter("is_clockwise", isClockwise).value
    gainLeft = node.declare_parameter("gain_left", gainLeft).value
    gainRight = node.declare_parameter("gain_right", gainRight).value

    if isClockwise:
        signLeft = -1.0
        signRight = 1.0
    else:
        signLeft = 1.0
        signRight = -1.0

    vel_gain = node.declare_parameter("vel_gain", 1.0).value
    vel_integrator_gain = node.declare_parameter("vel_integrator_gain", 10.0).value

    encoder_bandwidth = node.declare_parameter("encoder_bandwidth", 200).value
    motor_bandwidth = node.declare_parameter("motor_bandwidth", 200).value

    wtimer =node.declare_parameter("wd_timeout", 1.0).value
    wait_first_command = node.declare_parameter("wait_first_command", True).value  # does not set watchdog timer before receiving first motorTarget input.
    reset_watchdog_error = node.declare_parameter("reset_watchdog", True).value  # reset watchdog timeout error at start-up.
    connection_timeout = Duration(seconds=node.declare_parameter("connection_timeout", 5.0).value)
    fw_version_str = node.declare_parameter("fw_version", "").value
    
    odrive_left_serial_str = None
    odrive_right_serial_str = None
    try:
        odrive_left_serial_str = node.declare_parameter("odrive_left_serial_number", rclpy.Parameter.Type.STRING).value
    except:
        odrive_left_serial_str = str(node.declare_parameter("odrive_left_serial_number", rclpy.Parameter.Type.INTEGER).value)
    try:
        odrive_right_serial_str = node.declare_parameter("odrive_right_serial_number", rclpy.Parameter.Type.STRING).value
    except:
        odrive_right_serial_str = str(node.declare_parameter("odrive_right_serial_number", rclpy.Parameter.Type.INTEGER).value)
    if odrive_left_serial_str == "" or odrive_right_serial_str == "":
        logger.error("CABOT_ODRIVER_SERIAL_0 and 1 should be specified in .env file!!")
        return

    ## Diagnostic Updater
    updater = Updater(node)
    updater.add(TopicCheckTask("Motor Target", "/cabot/motorTarget", MotorTarget, MotorTargetRoutine))
    updater.add(OdriveDeviceTask("ODrive"))

    find_controller(0, odrive_left_serial_str)
    find_controller(1, odrive_right_serial_str)

    # adjust motor configuration
    def set_config(odrv_index):
        try:
            odrvs[odrv_index].axis0.controller.config.vel_gain = vel_gain
            odrvs[odrv_index].axis0.controller.config.vel_integrator_gain = vel_integrator_gain
            odrvs[odrv_index].axis0.config.encoder_bandwidth = encoder_bandwidth
            odrvs[odrv_index].axis0.config.motor.current_control_bandwidth = motor_bandwidth
        except:
            logger.error("Can not set motor configuration!!")
            return
    set_config(0)
    set_config(1)

    last_feed = 0

    rate = node.create_rate(freq)
    ms = MotorStatus()

    mode_written=None
    spd0_c_written,spd1_c_written=None,None

    def stop_control():
        od_writeSpd(0,0)
        od_writeSpd(1,0)
        od_setWatchdogTimer(0,0)
        od_setWatchdogTimer(0,1)
        od_writeMode(0)

    # variables to manage connection error
    odrv_is_active = True
    time_disconnect = node.get_clock().now()

    while rclpy.ok:
        # retry connection after timeout
        if not odrv_is_active:
            diff_time = node.get_clock().now() - time_disconnect
            if diff_time > connection_timeout:
                logger.warn("Odrive connection timeout. Retry finding odrive contoller...")
                time_disconnect = node.get_clock().now()
                find_controller(path, clear=True, reset_watchdog_error=reset_watchdog_error)
                set_config(0)
                set_config(1)

        if version_mismatched or (use_index and index_not_found):
            if odrv_is_active:
                time_disconnect = node.get_clock().now()
            continue

        # check odrv remote object
        try:
            odrvs[0].axis0
            odrvs[1].axis0 
        except:
            # if changes from True to False
            if odrv_is_active:
                time_disconnect = node.get_clock().now()

            odrv_is_active = False

            # reset written values
            mode_written = None
            spd0_c_written , spd1_c_written = None , None

            import traceback
            logger.error("Failed to access odrv axes.", throttle_duration_sec=5.0)
            rate.sleep()
            continue
        else:
            # recovery from inactive
            if not odrv_is_active:
                try:
                    # reset odrv0 control
                    stop_control()
                    if reset_watchdog_error:
                        reset_error_watchdog_timer_expired(0)
                        reset_error_watchdog_timer_expired(1)
                        rate.sleep()
                except:
                    import traceback
                    logger.error("Failed to reset odrv control.")
                    logger.error(traceback.format_exc())
                    rate.sleep()
                    continue
                else:
                    odrv_is_active = True

        # error check
        try: 
            if _odrv_has_error(odrvs[0]) or _odrv_has_error(odrvs[1]):
                odrive_error_str = \
                    "\nErrors of odrv0 \n" + str(format_errors(odrvs[0])) + \
                    "\nErrors of odrv1 \n" + str(format_errors(odrvs[1])) 
                logger.error(odrive_error_str, throttle_duration_sec=1.0)
                time_disconnect = node.get_clock().now()
                od_writeMode(0)
                mode_written = None
                spd0_c_written , spd1_c_written = None , None
                rate.sleep()
                continue
        except:
            import traceback
            exception_string = traceback.format_exc()
            logger.error(exception_string)
            rate.sleep()
            continue


        # send new velocity command
        global lock
        lock.acquire()
        try:
            if(mode_written!=loopCtrl_on):

                if PRINTDEBUG: print('w m ', loopCtrl_on)
                if od_writeMode(loopCtrl_on):
                    mode_written=loopCtrl_on

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
            logger.error("Failed to set requested_state and vel_setpoint")
            logger.error(exception_string)
            rate.sleep()
            continue
        
        # set watchdog timer
        try:
            # enable watchdog timer after receiving at least one motorTarget
            if wait_first_command:
                if count_motorTarget is not None:
                    od_setWatchdogTimer(wtimer,0)
                    od_setWatchdogTimer(wtimer,1)
            else:
                od_setWatchdogTimer(wtimer,0)
                od_setWatchdogTimer(wtimer,1)


            # feed watchdog timer
            if count_motorTarget is not None:
                if count_motorTarget != previous_count_motorTarget:
                    # call watchdog_feed only when motorTarget is being updated. odrive motors stop when motorTarget update stops.
                    od_feedWatchdogTimer(0)
                    od_feedWatchdogTimer(1)
                    previous_count_motorTarget = count_motorTarget
        except:
            import traceback
            exception_string = traceback.format_exc()
            logger.error("Failed to set watchdog timer")
            logger.error(exception_string)
            rate.sleep()
            continue

        enc0 = enc1 = spd0 = spd1 = None
        current_setpoint_0 = current_setpoint_1 = None
        current_measured_0 = current_measured_1 = None
        # update encoder counts and speed
        try:
            enc0, spd0 = odrvs[0].axis0.pos_vel_mapper.pos_rel, odrvs[0].axis0.pos_vel_mapper.vel#getFloats(getResponse("f 0"))
            enc1, spd1 = odrvs[1].axis0.pos_vel_mapper.pos_rel, odrvs[1].axis0.pos_vel_mapper.vel#getFloats(getResponse("f 1"))
            current_setpoint_0 = odrvs[0].axis0.motor.foc.Iq_setpoint
            current_setpoint_1 = odrvs[1].axis0.motor.foc.Iq_setpoint
            current_measured_0 = odrvs[0].axis0.motor.foc.Iq_measured
            current_measured_1 = odrvs[1].axis0.motor.foc.Iq_measured
        except:
            print("Reading TRY failed!")
            rate.sleep()
            import traceback
            exception_string = traceback.format_exc()
            logger.error(exception_string)
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
    od_setWatchdogTimer(0,0)
    od_setWatchdogTimer(0,1)
    od_writeMode(0)

def od_reboot():
    odrvs[0].reboot()
    odrvs[1].reboot()

def od_writeSpd(ch, spd):
    try:
        ctrl = odrvs[ch].axis0.controller
        ctrl.config.control_mode = ControlMode.VELOCITY_CONTROL
        ctrl.input_vel = spd
        return 1
    except:
        raise

def od_setWatchdogTimer(sec, odrv_index):
    if 0 < sec:
        # store previous watchdog_timeout values to use them later
        prev_watchdog_timeout = odrvs[odrv_index].axis0.config.watchdog_timeout

        odrvs[odrv_index].axis0.config.watchdog_timeout = sec

        # if previous watchdog_timeout == 0, reset watchdog timer by watchdog_feed to prevent immediate timeout.
        # watchdog_feed must be called after watchdog_timeout is set because watchdog_timeout seems to be used in watchdog_feed function.
        if prev_watchdog_timeout == 0:
            odrvs[odrv_index].axis0.watchdog_feed()

        odrvs[odrv_index].axis0.config.enable_watchdog = True
    else:
        # disable watchdog timer before setting watchdog_timeout to 0
        odrvs[odrv_index].axis0.config.enable_watchdog = False
        odrvs[odrv_index].axis0.config.watchdog_timeout = sec

def od_feedWatchdogTimer(odrv_index):
    odrvs[odrv_index].axis0.watchdog_feed()

def od_writeMode(loopCtrl_on):
    try:
        if (loopCtrl_on==1):
            odrvs[0].axis0.requested_state = AxisState.CLOSED_LOOP_CONTROL
            odrvs[1].axis0.requested_state = AxisState.CLOSED_LOOP_CONTROL
            return 1
        else:
            odrvs[0].axis0.requested_state = AxisState.IDLE
            odrvs[1].axis0.requested_state = AxisState.IDLE
            return 1
    except:
        raise

'''Run'''
if __name__ == '__main__':
    try:
        rclpy.init()
        node = rclpy.create_node('odrive_s1_node')
        logger = node.get_logger()

        thread = threading.Thread(target=main, daemon=True)
        thread.start()
        rclpy.spin(node)

    except ROSInterruptException:
        pass