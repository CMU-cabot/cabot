#!/usr/bin/env python3

# Copyright (c) 2022  Carnegie Mellon University
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

import code
import enum
import logging
import math
import os

import rclpy
from tf_transformations import euler_from_quaternion

import cabot_ui.geoutil
from cabot_ui.cabot_rclpy_util import CaBotRclpyUtil


class DummyTime:
    def __init__(self):
        self.time = None

    def now(self):
        return self.time


class Dummy:
    def __init__(self):
        self.Time = DummyTime()

    def update_time(self, time):
        self.Time.time = time


reading_rosbag = False
dummy = None


def now():
    global dummy
    if dummy:
        return dummy.Time.now()

    try:
        return CaBotRclpyUtil.now()
    except:  # noqa: E722
        if not dummy:
            dummy = Dummy()
        return dummy.Time.now()


def now_str():
    if dummy:
        time = now()
        return F"{time.sec:9d}.{time.nanosec:09d}"

    return F"{time.seconds:9d}.{time.nanoseconds:09d}"


def time_zero():
    if dummy:
        return dummy.Time.now()
    return rclpy.time.Time(0, 0, CaBotRclpyUtil.initialize().clock.clock_type)


logging.basicConfig(format='%(name)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG if 'DEBUG' in os.environ else logging.INFO)


class AbstractFilter:
    def __init__(self, max_duration):
        self.max_duration = max_duration
        self.history = []

    def input(self, time, value):
        self.history.append((time, value))
        while len(self.history) > 0 and (time - self.history[0][0]).to_sec() > self.max_duration:
            self.history.pop(0)

    def clear(self):
        self.history.clear()

    @property
    def latest(self):
        if len(self.history) == 0:
            return -1
        time, value = self.history[-1]
        if (now() - time).to_sec() > self.max_duration:
            return -1
        return value

    @property
    def duration_since_latest(self):
        if len(self.history) == 0:
            return 0
        time, _ = self.history[-1]
        return (now() - time).to_sec()


class EnumFilter(AbstractFilter):
    def __init__(self, max_duration):
        super().__init__(max_duration)

    @property
    def majority(self):
        vote = {}
        for (time, value) in self.history:
            if (now() - time).to_sec() > self.max_duration:
                continue
            if value in vote:
                vote[value] += 1
            else:
                vote[value] = 1
        maxvote = None
        maxcount = 0
        for key in vote.keys():
            if maxcount < vote[key]:
                maxcount = vote[key]
                maxvote = key
        return maxvote


class AverageFilter(AbstractFilter):
    def __init__(self, max_duration):
        super().__init__(max_duration)

    @property
    def average(self):
        time = now()
        ave = 0
        count = 0
        for (t, value) in self.history:
            if (time - t).to_sec() > self.max_duration:
                continue
            ave += value
            count += 1
        if count == 0:
            return None
        return ave / count

    @property
    def minimum(self):
        if len(self.history) == 0:
            return None
        ret = self.history[0][1]
        for (t, value) in self.history:
            if value < ret:
                ret = value
        return ret


class StopReason(enum.Enum):
    UNKNOWN = enum.auto()
    NO_NAVIGATION = enum.auto()
    NOT_STOPPED = enum.auto()
    NO_ODOMETORY = enum.auto()
    NO_TOUCH = enum.auto()
    STOPPED_BUT_UNDER_THRESHOLD = enum.auto()
    NO_CMD_VEL = enum.auto()
    THERE_ARE_PEOPLE_ON_THE_PATH = enum.auto()
    WAITING_FOR_ELEVATOR = enum.auto()
    AVOIDING_OBSTACLE = enum.auto()
    AVOIDING_PEOPLE = enum.auto()


class StopReasonFilter():
    def __init__(self):
        self.prev_code = None
        self.prev_event_duration = 0
        self.prev_summary_duration = 0
        self.code = None
        self.duration = 0
        self.event_interval = 0.5
        self.summary_interval = 15.0

    def update(self, duration, code):
        self.duration = duration
        self.code = code

    def conclude(self):
        self.prev_code = self.code
        self.code = None
        self.duration = 0

    def event(self):
        if not code:
            return (0, None)
        if self.prev_code != self.code or \
           self.duration - self.prev_event_duration > self.event_interval:
            self.prev_event_duration = self.duration
            return (self.duration, self.code)
        return (0, None)

    def summary(self):
        if self.code not in [StopReason.NO_NAVIGATION, StopReason.NO_TOUCH,
                             StopReason.NOT_STOPPED, StopReason.STOPPED_BUT_UNDER_THRESHOLD]:
            if self.prev_code != self.code or \
               self.duration - self.prev_summary_duration > self.summary_interval:
                self.prev_summary_duration = self.duration
                return (self.duration, self.code)
        return (0, None)


class StopReasoner:
    FILTER_DURATION = 1.0
    LONG_FILTER_DURATION = 5.0
    STOP_DURATION_THRESHOLD = 3.0
    REPLAN_REASON_COUNT = 2
    STOP_LINEAR_VELOCITY_THRESHOLD = 0.2
    STOP_ANGULAR_VELOCITY_THRESHOLD = 0.2

    def __init__(self, tf_transformer):
        self.tf_transformer = tf_transformer
        self.stopped = False
        self.stopped_time = None

        self._navigating = False
        self._waiting_for_elevator = False
        self.last_log = None
        self.current_frame = None

        self.linear_velocity = AverageFilter(StopReasoner.FILTER_DURATION)
        self.angular_velocity = AverageFilter(StopReasoner.FILTER_DURATION)
        self.cmd_vel_linear = AverageFilter(StopReasoner.FILTER_DURATION)
        self.cmd_vel_angular = AverageFilter(StopReasoner.FILTER_DURATION)

        self.people_speed = AverageFilter(StopReasoner.LONG_FILTER_DURATION)
        self.touch_speed = AverageFilter(StopReasoner.LONG_FILTER_DURATION)

        self.replan_reason = EnumFilter(StopReasoner.LONG_FILTER_DURATION)

    def clear_history(self):
        # self.linear_velocity.clear()
        # self.angular_velocity.clear()
        # self.cmd_vel_linear.clear()
        # self.cmd_vel_angular.clear()
        self.people_speed.clear()
        self.touch_speed.clear()
        self.replan_reason.clear()

    def update_time(self, msg):
        global dummy
        if not dummy:
            dummy = Dummy()
        dummy.update_time(msg)

    def input_event(self, msg):
        if msg.data.startswith("button"):
            return
        if msg.data.startswith("click"):
            return

        if msg.data.startswith("navigation"):
            logger.debug("%.2f, %s", now().to_sec(), msg.data)

        if msg.data == "navigation;event;navigation_start":  # to be fixed with event class
            self.navigating = True
        elif msg.data == "navigation;event;waiting_for_elevator":
            self._waiting_for_elevator = True
        elif msg.data == "navigation;event;elevator_door_may_be_ready":
            self._waiting_for_elevator = True
            # this could be issued before the door opens
            pass

        """
        elif msg.data == "navigation_arrived":  # to be fixed with event class
            self.navigating = False
        elif msg.data == "navigation_pause":  # to be fixed with event class
            self.navigating = False
        elif msg.data == "navigation_resume;":  # to be fixed with event class
            self.navigating = True
        elif msg.data== "navigation_cancel":
            self.navigating = False
        elif msg.data== "navigation_next":
            pass
        elif msg.data.startswith("navigation_destination"):
            pass
        elif msg.data == "navigation;event;elevator_door_may_be_ready":
            self.navigating = True
            logger.debug("waited elevator")
        else:
            logger.debug("Not processed %s", msg.data)
        """

    def input_goal_topic(self, msg):
        # this is too early, robot needs to wait for the actual start
        # self.navigating = True
        pass

    def input_result_topic(self, msg):
        self.navigating = False
        self._waiting_for_elevator = False

    def input_current_frame(self, msg):
        self.current_frame = msg.data

    @property
    def navigating(self):
        return self._navigating

    @navigating.setter
    def navigating(self, newValue):
        if newValue != self._navigating:
            logger.debug("%.2f, %s", now().to_sec(), "Navigation Started" if newValue else "Navigation Stopped")
        self._navigating = newValue
        if newValue is False:
            self.clear_history()

    @property
    def waiting_for_elevator(self):
        return self._waiting_for_elevator

    def input_odom(self, msg):
        lx = msg.twist.twist.linear.x
        ly = msg.twist.twist.linear.y
        az = msg.twist.twist.angular.z
        self.linear_velocity.input(now(), abs(math.sqrt(lx*lx + ly*ly)))
        self.angular_velocity.input(now(), abs(az))

    def input_people_speed(self, msg):
        self.people_speed.input(now(), msg.data)

    def input_touch_speed(self, msg):
        self.touch_speed.input(now(), msg.data)

    def input_cmd_vel(self, msg):
        self.cmd_vel_linear.input(now(), abs(msg.linear.x))
        self.cmd_vel_angular.input(now(), abs(msg.angular.z))

    def input_replan_reason(self, msg):
        try:
            transform = self.tf_transformer.lookupTransform(self.current_frame, "base_footprint", time_zero())
            _, _, yaw = euler_from_quaternion(transform[1])
            logger.debug(transform)

            p0 = cabot_ui.geoutil.Point(x=transform[0][0], y=transform[0][1])
            p1 = cabot_ui.geoutil.Point(x=msg.position.x, y=msg.position.y)
            q = cabot_ui.geoutil.q_from_points(p1, p0)
            yaw = cabot_ui.geoutil.get_yaw(q)

            if p0.distance_to(p1) > 3:
                logger.debug("too far")
                return

            if not cabot_ui.geoutil.diff_in_angle(transform[1], q, 90):
                logger.debug("not in angle")
                return

            if len(msg.tags) == 0:
                return

            logger.debug("%.2f, %.2f) %.2f", transform[0][0], transform[0][1], yaw)
            logger.debug("(%.2f, %.2f) %.2f", msg.position.x, msg.position.y, yaw)
            logger.debug("%.2f, %s", now().to_sec(), msg.tagnames[0])
            if msg.tagnames[0] == "avoiding obstacle":
                self.replan_reason.input(now(), StopReason.AVOIDING_OBSTACLE)
            if msg.tagnames[0] == "avoiding people":
                self.replan_reason.input(now(), StopReason.AVOIDING_PEOPLE)
        except RuntimeError as e:
            CaBotRclpyUtil.error(e)
        except Exception as e:
            CaBotRclpyUtil.error(e)

    def update(self):
        if not self.navigating:
            self.stopped = False
            self.stopped_time = None
            return (0, StopReason.NO_NAVIGATION)

        ts_latest = self.touch_speed.latest
        ts_average = self.touch_speed.average
        if ts_latest >= 0 and ts_average is not None and \
           (ts_latest == 0 or ts_average < 1.0):
            return (0, StopReason.NO_TOUCH)

        # average velocity is under threshold
        if self.linear_velocity.latest < StopReasoner.STOP_LINEAR_VELOCITY_THRESHOLD and \
           self.angular_velocity.latest < StopReasoner.STOP_ANGULAR_VELOCITY_THRESHOLD:
            if not self.stopped:
                self.stopped_time = now()
                self.stopped = True
        else:
            self.stopped = False
            self.stopped_time = None

        if self.stopped_time:
            duration = (now() - self.stopped_time).to_sec()
        else:
            duration = -1

        # if self.linear_velocity.latest < 0 or \
        #    self.angular_velocity.latest < 0:
        #     return (self.linear_velocity.duration_since_latest, StopReason.NO_ODOMETORY)

        if not self.stopped:
            return (0, StopReason.NOT_STOPPED)

        if duration < StopReasoner.STOP_DURATION_THRESHOLD:
            return (duration, StopReason.STOPPED_BUT_UNDER_THRESHOLD)

        if self.cmd_vel_linear.latest < -1:
            return (duration, StopReason.NO_CMD_VEL)

        # if self.people_speed.minimum and self.people_speed.minimum < StopReasoner.STOP_LINEAR_VELOCITY_THRESHOLD:
        if self.people_speed.minimum is not None:
            if self.people_speed.minimum < 0.9:
                logger.debug("%.2f, people_speed minimum=%.2f, average=%.2f", now().to_sec(), self.people_speed.minimum, self.people_speed.average)
                return (duration, StopReason.THERE_ARE_PEOPLE_ON_THE_PATH)

        if self.waiting_for_elevator:
            return (duration, StopReason.WAITING_FOR_ELEVATOR)

        if self.replan_reason.majority is not None:
            return (duration, self.replan_reason.majority)

        return (duration, StopReason.UNKNOWN)
