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

from enum import Enum
from threading import Lock
from cabot_ui.cabot_rclpy_util import CaBotRclpyUtil


class State(Enum):
    # nothing is happening, may show menu
    idle = 0
    # transition from idle or in_pause to waiting_action
    waiting_action = 1
    # doing something, but need to ask NavCog Cabot node to know which action is executing
    in_action = 2
    # transition from in_action to in_pause
    waiting_pause = 3
    # trying to pause the goal
    in_pausing = 4
    # pausing action to wait user's decision
    in_pause = 5
    # summons
    in_summons = 6


class StatusManager(object):
    _unique_instance = None
    _lock = Lock()

    def __new__(cls):
        raise NotImplementedError('Cannot initialize via Constructor')

    @classmethod
    def __internal_new__(cls):
        return super(StatusManager, cls).__new__(cls)

    @classmethod
    def get_instance(cls):
        if not cls._unique_instance:
            with cls._lock:
                if not cls._unique_instance:
                    cls._unique_instance = cls.__internal_new__()
                    cls._unique_instance.__init__()
        return cls._unique_instance

    def __init__(self):
        # initialize
        self._current = State.idle

    @property
    def state(self):
        return self._current

    def set_state(self, state):
        CaBotRclpyUtil.info(F"NavigationState: changing from {self._current} to {state}")
        self._current = state
