# Copyright (c) 2023  Carnegie Mellon University, IBM Corporation, and others
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

import queue
import threading

import rospy

from cabot_ui import i18n
from cabot_ui.interface import UserInterface
from cabot_ui.navigation_explore import NavigationExploreInterface

class UserInterfaceExplore(UserInterface, NavigationExploreInterface):
    def __init__(self):
        super(UserInterfaceExplore, self).__init__()

        self._min_speak_interval = 2.0
        self._speak_text_queue = queue.Queue()
        self._speak_thread = None
        self._last_speak_time = None

    def _speak_with_interval(self, text):
        self._speak_text_queue.put(text)

        if (self._speak_thread is None) or (not self._speak_thread.is_alive()):
            self._speak_thread = threading.Thread(target=self._start_speak_queued_texts)
            self._speak_thread.start()

    def _start_speak_queued_texts(self):
        while not self._speak_text_queue.empty():
            r = rospy.Rate(10)
            while (self._last_speak_time is not None) and ((rospy.Time.now()-self._last_speak_time).to_sec()<self._min_speak_interval):
                r.sleep()

            text = self._speak_text_queue.get()
            self.speak(text)
            self._last_speak_time = rospy.Time.now()

    ## navigate interface
    def start_explore(self):
        self._speak_with_interval(i18n.localized_string("EXPLORE_START"))

    def start_explore_forward(self):
        self._speak_with_interval(i18n.localized_string("EXPLORE_FORWARD_START"))

    def finish_explore_forward(self):
        self._speak_with_interval(i18n.localized_string("EXPLORE_FORWARD_FINISH"))

    def go_origin(self):
        self._speak_with_interval(i18n.localized_string("EXPLORE_GO_ORIGIN"))

    def arrive_origin(self):
        self._speak_with_interval(i18n.localized_string("EXPLORE_ARRIVE_ORIGIN"))

    def system_cancel_explore(self):
        self._speak_with_interval(i18n.localized_string("EXPLORE_CANCEL"))

    def user_cancel_explore(self):
        self._speak_with_interval(i18n.localized_string("EXPLORE_CANCEL"))

    def find_forward_right_left_route(self):
        self._speak_with_interval(i18n.localized_string("EXPLORE_FIND_ROUTE").format(i18n.localized_string("EXPLORE_SEPARATOR").join([i18n.localized_string("EXPLORE_FORWARD"), i18n.localized_string("RIGHT"), i18n.localized_string("LEFT")])))

    def find_forward_right_route(self):
        self._speak_with_interval(i18n.localized_string("EXPLORE_FIND_ROUTE").format(i18n.localized_string("EXPLORE_SEPARATOR").join([i18n.localized_string("EXPLORE_FORWARD"), i18n.localized_string("RIGHT")])))

    def find_forward_left_route(self):
        self._speak_with_interval(i18n.localized_string("EXPLORE_FIND_ROUTE").format(i18n.localized_string("EXPLORE_SEPARATOR").join([i18n.localized_string("EXPLORE_FORWARD"), i18n.localized_string("LEFT")])))

    def find_right_left_route(self):
        self._speak_with_interval(i18n.localized_string("EXPLORE_FIND_ROUTE").format(i18n.localized_string("EXPLORE_SEPARATOR").join([i18n.localized_string("RIGHT"), i18n.localized_string("LEFT")])))

    def find_right_route(self):
        self._speak_with_interval(i18n.localized_string("EXPLORE_FIND_ROUTE").format(i18n.localized_string("RIGHT")))

    def find_left_route(self):
        self._speak_with_interval(i18n.localized_string("EXPLORE_FIND_ROUTE").format(i18n.localized_string("LEFT")))

    def find_dead_end(self):
        self._speak_with_interval(i18n.localized_string("EXPLORE_FIND_DEAD_END"))

    def move_forward_route(self):
        self._speak_with_interval(i18n.localized_string("EXPLORE_MOVE_FORWARD"))

    def turn_right_route(self):
        self._speak_with_interval(i18n.localized_string("EXPLORE_TURN").format(i18n.localized_string("RIGHT")))

    def turn_left_route(self):
        self._speak_with_interval(i18n.localized_string("EXPLORE_TURN").format(i18n.localized_string("LEFT")))

    def cannot_move_forward_route(self):
        self._speak_with_interval(i18n.localized_string("EXPLORE_CANNOT_TURN").format(i18n.localized_string("EXPLORE_FORWARD")))

    def cannot_turn_right_route(self):
        self._speak_with_interval(i18n.localized_string("EXPLORE_CANNOT_TURN").format(i18n.localized_string("RIGHT")))

    def cannot_turn_left_route(self):
        self._speak_with_interval(i18n.localized_string("EXPLORE_CANNOT_TURN").format(i18n.localized_string("LEFT")))

    def cannot_turn_route(self):
        self._speak_with_interval(i18n.localized_string("EXPLORE_CANNOT_TURN_ROUTE"))

    def find_intersection_routes_in_clock_direction(self, paths):
        self._speak_with_interval(i18n.localized_string("EXPLORE_FIND_ROUTE").format(i18n.localized_string("EXPLORE_SEPARATOR").join([i18n.localized_string(path) for path in paths])))

    def sign_reco(self):
        self._speak_with_interval(i18n.localized_string("RUN_SIGN_RECO"))
    
    def finding_intersection(self):
        self._speak_with_interval(i18n.localized_string("FINDING_INTERSECTION"))

    def error(self):
        pass
