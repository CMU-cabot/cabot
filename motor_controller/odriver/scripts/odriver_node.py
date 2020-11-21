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
import rospy
from odriver_msgs.msg import MotorStatus
from odriver_msgs.msg import MotorTarget
from std_msgs.msg import Header


'''Functional Imports'''
#import serial
import odrive
from odrive.utils import dump_errors
from odrive.enums import errors, CTRL_MODE_VELOCITY_CONTROL
import time
import numpy as np
import threading

PRINTDEBUG=False

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

'''Functional Constants (DON'T CHANGE)'''
AXIS_STATE_IDLE = 1
AXIS_STATE_CLOSED_LOOP_CONTROL = 8


'''Global Varaible'''
spd0_c, spd1_c = 0, 0
loopCtrl_on = 0
odrv0 = None
count_motorTarget = None

def find_controller(port, clear=False):
    '''Hardware Initialization'''
    global odrv0

    if clear:
        odrv0 = None

    while odrv0 is None and not rospy.is_shutdown():
        try:
            rospy.loginfo("Finding Orive controller...")
            odrv0 = odrive.find_any() if port is None else odrive.find_any(path=port)
            #od=serial.Serial(port, 115200,
            #                 timeout=serialReading_timeout,
            #                 write_timeout=serialWriting_timeout)
        except:
            rospy.logerr("Check Odrive connection: " + str(port) +  " doesn't exist! ")
            time.sleep(1)
        else:
            print "Odrive connected as ",odrv0.serial_number #odrv0.name, " !"
            rospy.loginfo("Odrive connected as " + str(odrv0.serial_number))


'''Subscriber Routine'''
def MotorTargetRoutine(data):
    global spd0_c
    global spd1_c
    global loopCtrl_on
    global lock
    global count_motorTarget
    lock.acquire()
    loopCtrl_on = data.loopCtrl
    spd0_c = signLeft * gainLeft * data.spdLeft / meter_per_count
    spd1_c = signRight * gainRight * data.spdRight / meter_per_count
    count_motorTarget = data.header.seq
    print spd0_c, spd1_c
    if leftIs1:
        spd0_c, spd1_c = spd1_c, spd0_c
    lock.release()


'''Main()'''
def main():
    path = rospy.get_param("~path", None)#specify path(e.g. usb:0001:0008) from .launch file, but not yet tested _aksg
    find_controller(path)
    od_setWatchdogTimer(0)

    rospy.init_node('odrive_node', anonymous=True)

    pub = rospy.Publisher('motorStatus', MotorStatus, queue_size=10)
    rospy.Subscriber('motorTarget', MotorTarget, MotorTargetRoutine, queue_size=10)

    global meter_per_count, leftIs1, signLeft, signRight, gainLeft, gainRight
    wheel_diameter = rospy.get_param("~wheel_diameter")
    count_per_round = rospy.get_param("~count_per_round")
    meter_per_count = wheel_diameter * np.pi / count_per_round
    if rospy.has_param("~left_is_1"): leftIs1 = rospy.get_param("~left_is_1")
    if rospy.has_param("~gain_left"): gainLeft = rospy.get_param("~gain_left")
    if rospy.has_param("~gain_right"): gainRight = rospy.get_param("~gain_right")

    vel_gain = rospy.get_param("~vel_gain", 0.002)
    vel_integrator_gain = rospy.get_param("~vel_integrator_gain", 0.015)
    encoder_bandwidth = rospy.get_param("~encoder_bandwidth", 3000)
    motor_bandwidth = rospy.get_param("~motor_bandwidth", 3000)

#    port = rospy.get_param("~port", '/dev/ttyODRIVE')# to be revised maybe serial number
#    find_controller(port)
#    wtimer = 0
    wtimer =rospy.get_param("~wd_timeout", 1.0)
    wait_first_command = rospy.get_param("~wait_first_command", True) # does not set watchdog timer before receiving first motorTarget input.
    reset_watchdog_error = rospy.get_param("~reset_watchdog", True) # reset watchdog timeout error at start-up.
    connection_timeout = rospy.get_param("~connection_timeout", 10.0)

    # fuction to convert errorcode to a list of error name
    def errorcode_to_list(error_code):
        error_list = []
        error_codes_tup =  [(name, value) for name, value in errors.axis.__dict__.items() if "ERROR_" in name]
        for codename, codeval in error_codes_tup:
            if error_code & codeval != 0:
                error_list.append(codename)
        return error_list

    # if an axis is stopped by watchdog timeout, reset the error status.
    def reset_error_watchdog_timer_expired():
        if odrv0.axis0.error == errors.axis.ERROR_WATCHDOG_TIMER_EXPIRED:
            odrv0.axis0.error = errors.axis.ERROR_NONE
        if odrv0.axis1.error == errors.axis.ERROR_WATCHDOG_TIMER_EXPIRED:
            odrv0.axis1.error = errors.axis.ERROR_NONE

    if reset_watchdog_error:
        reset_error_watchdog_timer_expired()

    # adjust motor configuration
    odrv0.axis0.controller.config.vel_gain = vel_gain
    odrv0.axis1.controller.config.vel_gain = vel_gain
    odrv0.axis0.controller.config.vel_integrator_gain = vel_integrator_gain
    odrv0.axis1.controller.config.vel_integrator_gain = vel_integrator_gain
    odrv0.axis0.encoder.config.bandwidth = encoder_bandwidth
    odrv0.axis1.encoder.config.bandwidth = encoder_bandwidth
    odrv0.axis0.motor.config.current_control_bandwidth = motor_bandwidth
    odrv0.axis1.motor.config.current_control_bandwidth = motor_bandwidth

    last_feed = 0

    rate = rospy.Rate(freq)
    ms = MotorStatus()

    mode_written=None
    spd0_c_written,spd1_c_written=None,None

    #time.sleep(2)
    ## find index for debug
    #od_write('w axis0.requested_state 6')
    #time.sleep(4)
    #od_write('w axis1.requested_state 6')
    #time.sleep(4)

    def stop_control():
        od_writeSpd(0,0)
        od_writeSpd(1,0)
        od_setWatchdogTimer(0)
        od_writeMode(0)

    def shutdown_hook():
        stop_control()

    rospy.on_shutdown(shutdown_hook)

    # variables to manage connection error
    odrv0_is_active = True
    time_disconnect = rospy.Time.now()

    while not rospy.is_shutdown():

        # retry connection after timeout
        if not odrv0_is_active:
            diff_time = rospy.Time.now() - time_disconnect
            if diff_time.to_sec() > connection_timeout:
                rospy.logwarn("Odrive connection timeout. Retry finding odrive contoller...")
                find_controller(path, clear=True)

        # check odrv0 remote object
        try:
            odrv0.axis0
            odrv0.axis1
        except:
            # if changes from True to False
            if odrv0_is_active:
                time_disconnect = rospy.Time.now()

            odrv0_is_active = False

            # reset written values
            mode_written=None
            spd0_c_written,spd1_c_written=None,None

            import traceback
            exception_string = traceback.format_exc()
            rospy.logerr("Failed to access odrv0 axes.")
            rospy.logerr(exception_string)
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
                    exception_string = traceback.format_exc()
                    rospy.logerr("Failed to reset odrv0 control.")
                    rospy.logerr(exception_string)
                    rate.sleep()
                    continue
                else:
                    odrv0_is_active = True

        ## read error
        #getResponse("r axis0.motor.error", "error    0")
        #getResponse("r axis1.motor.error", "error    1")

        # error check
        try:
            if odrv0.axis0.error != errors.axis.ERROR_NONE or odrv0.axis1.error != errors.axis.ERROR_NONE:
                rospy.logerr("Motor controller error: odrv0.axis0.error="+str(errorcode_to_list(odrv0.axis0.error))+", odrv0.axis1.error="+str(errorcode_to_list(odrv0.axis1.error)))
                rospy.logerr(dump_errors(odrv0))
                rate.sleep()
                continue
        except:
            import traceback
            exception_string = traceback.format_exc()
            rospy.logerr(exception_string)
            rate.sleep()
            continue


        # send new velocity command
        global lock
        lock.acquire()
        try:
            if(mode_written!=loopCtrl_on):
                if PRINTDEBUG: print 'w m ', loopCtrl_on
                if od_writeMode(loopCtrl_on):
                    mode_written=loopCtrl_on

            if(spd0_c_written!=spd0_c):
                if PRINTDEBUG: print 'w 0 {:0.2f}'.format(spd0_c)
                if od_writeSpd(0,spd0_c):
                    spd0_c_written=spd0_c

            if(spd1_c_written!=spd1_c):
                if PRINTDEBUG: print 'w 1 {:0.2f}'.format(spd1_c)
                if od_writeSpd(1,spd1_c):
                    spd1_c_written=spd1_c
            lock.release()
        except:
            lock.release()
            import traceback
            exception_string = traceback.format_exc()
            rospy.logerr("Failed to set requested_state and vel_setpoint")
            rospy.logerr(exception_string)
            rate.sleep()
            continue

        # set watchdog timer
        try:
            # enable watchdog timer after receiving at least one motorTarget
            if wait_first_command:
                if count_motorTarget is not None:
                    od_setWatchdogTimer(wtimer)
            else:
                od_setWatchdogTimer(wtimer)
        except:
            import traceback
            exception_string = traceback.format_exc()
            rospy.logerr("Failed to set watchdog timer")
            rospy.logerr(exception_string)
            rate.sleep()
            continue

#        if 0 == wtimer:
#            wtimer = rospy.get_param("~wd_timeout", 1.0)
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
            rospy.sleep(0.001)
            print "Reading TRY failed!"
            rate.sleep()
            import traceback
            exception_string = traceback.format_exc()
            rospy.logerr(exception_string)
            continue

        if enc0 is not None and enc1 is not None and \
           spd0 is not None and spd1 is not None:
            ms.header.stamp = rospy.Time.now()

            if leftIs1:
                ms.distLeft_c  = enc1
                ms.distRight_c = enc0
                ms.spdLeft_c   = spd1
                ms.spdRight_c  = spd0

                ms.currentSetpointLeft = current_setpoint_1 * signLeft
                ms.currentSetpointRight = current_setpoint_0 * signRight
                ms.currentMeasuredLeft = current_measured_1 * signLeft
                ms.currentMeasuredRight = current_measured_0 * signRight
            else:
                ms.distLeft_c  = enc0
                ms.distRight_c = enc1
                ms.spdLeft_c   = spd0
                ms.spdRight_c  = spd1

                ms.currentSetpointLeft = current_setpoint_0 * signLeft
                ms.currentSetpointRight = current_setpoint_1 * signRight
                ms.currentMeasuredLeft = current_measured_0 * signLeft
                ms.currentMeasuredRight = current_measured_1 * signRight

            ms.distLeft_c  *= signLeft / gainLeft
            ms.distRight_c *= signRight / gainRight
            ms.spdLeft_c  *= signLeft / gainLeft
            ms.spdRight_c *= signRight / gainRight

            ms.distLeft  = ms.distLeft_c  * meter_per_count
            ms.distRight = ms.distRight_c * meter_per_count
            ms.spdLeft  = ms.spdLeft_c  * meter_per_count
            ms.spdRight = ms.spdRight_c * meter_per_count
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
        print "Illegal input:'%s'"%(text)
        return None, None
    try:
        return float(items[0]), float(items[1])
    except:
        print "Illegal input:'%s'"%(text)
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
        ctrl.config.control_mode = CTRL_MODE_VELOCITY_CONTROL
        ctrl.vel_setpoint = spd
        return 1
    except:
        raise

def od_setWatchdogTimer(sec):
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
    if PRINTDEBUG: print cmd, result
    return result

def od_writeMode(loopCtrl_on):
    try:
        if (loopCtrl_on==1):
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
def od_writeMode_obsolete(loopCtrl_on):
    if (loopCtrl_on==1):
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
        if PRINTDEBUG: print cmd.rstrip()
        if (odrv0.write(cmd)==length):
            return 1
    except:
        print "FAILED!!!! "+str(cmd)
    return 0

def getResponse(st, comment=None): # To be improved
    result = od_write(st)

    time.sleep(pause_between_commands)
    line=odrv0.readline().rstrip()
    if use_checksum:
        if not "*" in line:
            print "Response without checksum '{}'".format(line)
            return None

        print line
        res, cs = line.split("*")
        try:
            if checksum(res) != int(cs):
                print "checksum is different {} != {}".format(checksum(res), int(cs))
                return None
        except:
            print "checksum is not int"
            return None
    else:
        if PRINTDEBUG: print line
        res = line

    if comment:
        print "{}: {:20s} - written[{}]: {}, ".format(comment, res, result, st)
    return res

'''Run'''
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass




#
