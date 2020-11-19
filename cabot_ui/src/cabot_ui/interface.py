# Copyright (c) 2020 Carnegie Mellon University
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


import math
import rospy
import std_msgs.msg
from cabot_ui import tts, visualizer, geojson, i18n
from cabot.handle_v2 import Handle

class UserInterface(object):
    def __init__(self):
        self.visualizer = visualizer.instance
        self.note_pub = rospy.Publisher("/cabot/notification",
                                        std_msgs.msg.Int8, queue_size=10, latch=True)
        rospy.wait_for_service('/speak')

        self.lang = rospy.get_param("~language", "en")
        i18n.set_language(self.lang)

    def speak(self, text, pose=None, force=True, pitch=50, volume=50, rate=50):
        rospy.logdebug("speak %s (%s) %s", text.encode('utf-8'), self.lang, pose)
        tts.speak(text, force=force, pitch=pitch, volume=volume, rate=rate, lang=self.lang)
        if pose is not None:
            self.visualizer.spoken.append((pose, text, "speak"))
            self.visualizer.visualize()

    def vibrate(self, pattern=Handle.UNKNOWN, pose=None):
        rospy.logdebug("vibrate %d %s", pattern, pose)
        msg = std_msgs.msg.Int8()
        msg.data = pattern
        self.note_pub.publish(msg)
        
        if pose is not None:
            self.visualizer.spoken.append((pose, "VIB:"+Handle.get_name(pattern), "vib"))
            self.visualizer.visualize()


    ## menu interface 
    def menu_changed(self, menu=None, backed=False, usage=False):
        if menu is None:
            return

        if backed:
            self.speak(menu.title, force=True)
        
        self.speak(menu.description, force=not backed)
        if usage and menu.usage:
            self.speak("__pose__", force=False)
            self.speak(menu.usage, force=False, pitch=25)

    def pause_navigation(self):
        self.speak(i18n.localized_string("PAUSE_NAVIGATION"))

    def cancel_navigation(self):
        self.speak(i18n.localized_string("CANCEL_NAVIGATION"))

    def resume_navigation(self):
        self.speak(i18n.localized_string("RESUME_NAVIGATION"))

    def start_exploration(self):
        self.speak(i18n.localized_string("START_EXPLORATION"))


    ## navigate interface
    def i_am_ready(self):
        self.speak(i18n.localized_string("I_AM_READY"))

    def start_navigation(self):
        self.vibrate(Handle.FRONT)

    def notify_turn(self, turn=None, pose=None):
        if turn.angle < -math.pi/3:
            self.vibrate(Handle.RIGHT_TURN, pose=pose)
        elif turn.angle < -math.pi/8:
            self.vibrate(Handle.RIGHT_DEV, pose=pose)
        elif turn.angle > math.pi/3:
            self.vibrate(Handle.LEFT_TURN, pose=pose)
        elif turn.angle > math.pi/8:
            self.vibrate(Handle.LEFT_DEV, pose=pose)
            
    def notify_human(self, angle=0, pose=None):
        vibration = Handle.RIGHT_DEV
        if angle > 0:
            vibration = Handle.LEFT_DEV

        self.vibrate(pattern=vibration, pose=pose)
        self.speak(i18n.localized_string("AVOIDING_A_PERSON"))

    def have_arrived(self):
        self.speak(i18n.localized_string("YOU_HAVE_ARRIVED"))

    def approaching_to_poi(self, poi=None, pose=None):
        statement = poi.approaching_statement()
        if statement:
            self.speak(statement, pose)

    def request_user_action(self, poi=None, pose=None):
        statement = poi.request_user_action_statement()
        if statement:
            self.speak(statement, pose)

    def approached_to_poi(self, poi=None, pose=None):
        statement = poi.approached_statement()
        if statement:
            self.speak(statement, pose)

    def passed_poi(self, poi=None, pose=None):
        statement = poi.passed_statement()
        if statement:
            self.speak(statement, pose)
