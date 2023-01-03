# Copyright (c) 2020, 2022  Carnegie Mellon University
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

import rclpy.node
from rclpy.duration import Duration
from cabot_ui.turn_detector import Turn
from people_msgs.msg import People
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry


class SocialNavigation(object):
    def __init__(self, node: rclpy.node.Node, buffer):
        self._node = node
        self._logger = node.get_logger()
        self._buffer = buffer
        self._path = None
        self._turn = None
        self._current_pose = None
        self._latest_odom = None
        self._latest_people = None
        self._latest_obstacles = None
        self._people_count = 0
        self._obstacles_count = 0
        self._event = None
        self._message = None
        self._last_message = None
        self._priority = 0
        self._last_message_time = 0
        self._last_category = None
        odom_topic = node.declare_parameter("odom_topic", "/odom").value
        people_topic = node.declare_parameter("people_topic", "/people").value
        obstacles_topic = node.declare_parameter("obstacles_topic", "/obstacles").value
        self.odom_topic = node.create_subscription(Odometry, odom_topic, self._odom_callback, 10)
        self.people_sub = node.create_subscription(People, people_topic, self._people_callback, 10)
        self.obstacles_sub = node.create_subscription(
            People, obstacles_topic, self._obstacles_callback, 10)

    def _odom_callback(self, msg):
        self._latest_odom = msg

    def _people_callback(self, msg):
        self._latest_people = msg

        if self._buffer is None:
            return

        count = 0
        for person in msg.people:
            point_stamped = PointStamped()
            point_stamped.header.frame_id = msg.header.frame_id
            point_stamped.point = person.position
            point_stamped = self._buffer.transform(point_stamped, "base_footprint")
            if abs(point_stamped.point.y) < 1.5 and \
               abs(point_stamped.point.y) < point_stamped.point.x and \
               0 < point_stamped.point.x and point_stamped.point.x < 5:
                count += 1

        self._people_count = count
        self._update()

    def _obstacles_callback(self, msg):
        self._latest_obstacles = msg

        if self._buffer is None:
            return

        count = 0
        # using person as obstacle
        for person in msg.people:
            point_stamped = PointStamped()
            point_stamped.header.frame_id = msg.header.frame_id
            point_stamped.point = person.position
            point_stamped = self._buffer.transform(point_stamped, "base_footprint")
            if abs(point_stamped.point.y) < 1.5 and \
               abs(point_stamped.point.y) < point_stamped.point.x and \
               0 < point_stamped.point.x and point_stamped.point.x < 5:
                count += 1

        self._obstacles_count = count
        self._update()

    def _update(self):
        self._logger.info(F"social navigation update turn={self._turn}", throttle_duration_sec=1)

        if self._turn is not None and self._turn.turn_type == Turn.Type.Avoiding:
            self._logger.info(F"avoiding turn, people count = {self._people_count}, "
                              F"obstacle count = {self._obstacles_count}")
            if self._people_count == 1:
                self._set_message("AVOIDING_A_PERSON", "AVOID", 10)
            elif self._people_count > 1:
                self._set_message("AVOIDING_PEOPLE", "AVOID", 10)
            elif self._obstacles_count == 1:
                self._set_message("AVOIDING_AN_OBSTACLE", "AVOID", 10)
            elif self._obstacles_count > 1:
                self._set_message("AVOIDING_OBSTACLES", "AVOID", 10)
            self._turn = None

        # check event
        if self._event is not None:
            param = self._event.param
            self._logger.info(F"event {param}")
            # trying to avoid people
            if param == "bt_navigator_avoid_person":  # trying to avoid people
                self._set_message("AVOIDING_A_PERSON", "AVOID", 10)
            if param == "bt_navigator_avoid_people":  # trying to avoid people
                self._set_message("AVOIDING_PEOPLE", "AVOID", 10)

            # stops due to people on the path
            if param == "people_speed_stopped":
                if self._people_count == 1:
                    self._set_message("A_PERSON_IN_THE_WAY", "IN_THE_WAY", 5)
                elif self._people_count > 1:
                    self._set_message("PEOPLE_IN_THE_WAY", "IN_THE_WAY", 5)

            # following people
            if param == "people_speed_following":
                if self._people_count == 1:
                    self._set_message("FOLLOWING_A_PERSON", "FOLLWING", 1)
                elif self._people_count > 1:
                    self._set_message("FOLLOWING_PEOPLE", "FOLLWING", 1)

            # delete event after check
            self._event = None

    def _set_message(self, message, category, priority):
        self._logger.nfo(F"set_message {message} {priority}")
        now = self._node.get_clock.now()
        if self._priority < priority and \
           (self._last_category != category or
                (now - self._last_message_time) > Duration(seconds=15.0)):
            self._priority = priority
            self._message = message
            self._last_category = category

    def get_message(self):
        now = self._node.get_clock().now()

        if self._message is not None and (now - self._last_message_time) > Duration(seconds=5.0):
            self._last_message_time = now
            self._last_message = self._message
            self._message = None
            self._priority = 0
            return self._last_message
        return None

    @property
    def path(self):
        return self._path

    @path.setter
    def path(self, path):
        self._path = path
        self._update()

    @path.deleter
    def path(self):
        del self._path

    @property
    def turn(self):
        return self._turn

    @turn.setter
    def turn(self, turn):
        self._turn = turn
        self._update()

    @turn.deleter
    def turn(self):
        del self._turn

    @property
    def event(self):
        return self._event

    @event.setter
    def event(self, event):
        self._event = event
        self._update()

    @event.deleter
    def event(self):
        del self._event

    @property
    def current_pose(self):
        return self._current_pose

    @current_pose.setter
    def current_pose(self, current_pose):
        self._current_pose = current_pose
        self._update()

    @current_pose.deleter
    def current_pose(self):
        del self._current_pose
