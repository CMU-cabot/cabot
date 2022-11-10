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

"""
Utility functions

Author: Daisuke Sato <daisukes@cmu.edu>
"""

import sys
import traceback

def callee_name():
    return sys._getframe().f_back.f_code.co_name

import threading
import time

# based on the following snipet
# https://stackoverflow.com/questions/5179467/equivalent-of-setinterval-in-python
def setInterval(interval, times=-1):
    """
    interval: interval for calling decorated function in seconds
    times: how many times do you want to call the function < 0 means infinite
    """

    # the interval should be greater than or equal to 0.001
    if interval < 0.001:
        raise RuntimeError("Interval should be greater than or equal to 0.001")


    def outer_wrap(function):
        """-"""
        #This will be the actual decorator, with fixed interval and times parameter

        def wrap(*args, **kwargs):
            """-"""
            #This will be the function to be called

            stop = threading.Event()

            # initial time
            start = time.time()

            def inner_wrap():
                """-"""
                # This is another function to be executed
                # in a different thread to simulate setInterval
                i = 0
                while i != times and not stop.isSet():
                    now = time.time()
                    i += 1

                    # check difference between now and the expected time
                    diff = start + interval*i - now
                    if diff < -interval:
                        #print "skip"
                        continue
                    stop.wait(diff)
                    try:
                        function(*args, **kwargs)
                    except:
                        print(traceback.format_exc())
                stop.set()

            timer = threading.Timer(0, inner_wrap)
            timer.daemon = True
            timer.start()
            return stop
        return wrap
    return outer_wrap
