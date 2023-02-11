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


from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time
from threading import Lock


# This utility class holds a node instance for convenience
class CaBotRclpyUtil(object):
    _unique_instance = None
    _lock = Lock()
    logger = None

    def __new__(cls, *args, **kargs):
        raise NotImplementedError('Cannot initialize via Constructor')

    @classmethod
    def __internal_new__(cls):
        return super(CaBotRclpyUtil, cls).__new__(cls)

    @classmethod
    def initialize(cls, node: Node = None):
        if not cls._unique_instance:
            with cls._lock:
                if node is None:
                    raise RuntimeError('initialize should be called with node for the first time')
                if not cls._unique_instance:
                    cls._unique_instance = cls.__internal_new__()
                    cls._unique_instance.__init__(node)
        return cls._unique_instance

    @classmethod
    def instance(cls):
        if not cls._unique_instance:
            raise RuntimeError('needs to be initialized first') 
        return cls._unique_instance

    def __init__(self, node: Node):
        # initialize
        self.logger = node.get_logger()
        self.clock = node.get_clock()

    @classmethod
    def info(cls, arg):
        cls.instance().logger.info(arg)

    @classmethod
    def debug(cls, arg):
        cls.instance().logger.debug(arg)

    @classmethod
    def error(cls, arg):
        cls.instance().logger.error(arg)

    @classmethod
    def fatal(cls, arg):
        cls.instance().logger.fatal(arg)

    @classmethod
    def warn(cls, arg):
        cls.instance().logger.warn(arg)

    @classmethod
    def now(cls):
        return cls.instance().clock.now()

    @classmethod
    def to_sec(cls, duration: Duration):
        return duration.nanoseconds / 1e9

    @classmethod
    def time_zero(cls):
        return Time(seconds=0, nanoseconds=0, clock_type=cls.instance().clock.clock_type)

