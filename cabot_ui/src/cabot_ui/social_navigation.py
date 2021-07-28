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

import rospy
from cabot_ui import event
from people_msgs.msg import People
from geometry_msgs.msg import PointStamped

class SocialNavigation(object):
    def __init__(self, listener):
        self._listener = listener
        self._path = None
        self._current_pose = None
        self._latest_people = None
        self._event = None
        self._message = None
        self._last_message = None
        self._priority = 0
        self._last_message_time = 0
        self._last_category = None
        people_topic = rospy.get_param("~people_topic", "/people")
        self.people_sub = rospy.Subscriber(people_topic, People, self._people_callback)

    def _people_callback(self, msg):
        self._latest_people = msg

        if self._listener is None:
            return

        count = 0
        for person in msg.people:
            point_stamped = PointStamped()
            point_stamped.header.frame_id = msg.header.frame_id
            point_stamped.point = person.position
            point_stamped = self._listener.transformPoint("base_footprint", point_stamped)
            if abs(point_stamped.point.y) < 2.0 and \
               0 < point_stamped.point.x and point_stamped.point.x < 10:
               count += 1

        self._people_count = count
        self._update()
        
    def _update(self):
        #rospy.loginfo("social navigation update")
        # check event
        if self._event is not None:
            param = self._event.param
            rospy.loginfo("event %s", param)
            # trying to avoid people
            if param == "bt_navigator_avoid_person": # trying to avoid people
                self._set_message("AVOIDING_A_PERSON", "AVOID", 10)
            if param == "bt_navigator_avoid_people": # trying to avoid people
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
        rospy.loginfo("set_message %s %d", message, priority)
        now = rospy.Time.now().to_sec()
        if self._priority < priority and \
           (self._last_category != category or (now - self._last_message_time) > 15.0):
            self._priority = priority
            self._message = message
            self._last_category = category

    def get_message(self):
        now = rospy.Time.now().to_sec()
        #rospy.loginfo("need_to_announce %s, %.2f, %.2f", self._message, now, self._last_message_time)
        if self._message is not None and (now - self._last_message_time) > 5.0:
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

