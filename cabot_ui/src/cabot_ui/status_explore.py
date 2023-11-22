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

import rospy
from enum import Enum
from threading import Lock

class StateExplore(Enum):
    # detecting intersection when initialized and stop
    processing = 0
    # user making decision which way to turn
    decision_making = 1
    # moving to next intersection
    moving_next_intersection = 2
    # moving to next cornet
    moving_next_corner = 3
    # moving to origin
    moving_origin = 4


class StatusManagerExplore(object):
    _unique_instance = None
    _lock = Lock()

    def __new__(cls):
        raise NotImplementedError('Cannot initialize via Constructor')
    
    @classmethod
    def __internal_new__(cls):
        return super(StatusManagerExplore, cls).__new__(cls)

    @classmethod
    def get_instance(cls):
        if not cls._unique_instance:
            with cls._lock:
                if not cls._unique_instance:
                    cls._unique_instance = cls.__internal_new__()
                    cls._unique_instance.__init__()
        return cls._unique_instance

    def __init__(self):
        #initialize
        self._current = StateExplore.processing
        self._turned_route = False

    @property
    def state(self):
        return self._current

    @property
    def turned_route(self):
        return self._turned_route

    @turned_route.setter
    def turned_route(self, value):
        self._turned_route = value

    def set_state(self, state):
        rospy.loginfo("NavigationState: changing from {} to {}".format(self._current, state))
        self._current = state
        if state==StateExplore.decision_making:
            self._turned_route = False