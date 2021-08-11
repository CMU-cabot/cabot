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

import os
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

        self.lang = rospy.get_param("~language", "en")
        self.site = rospy.get_param("~site", None)

        self.read_aloud = False
        if "CABOT_OPTION_READ_ALOUD_VIB_NOTIFICATION" in os.environ:
            if os.environ["CABOT_OPTION_READ_ALOUD_VIB_NOTIFICATION"] == "1":
                self.read_aloud = True
                rospy.loginfo("Read aloud vib notification is set to True")
            else:
                rospy.loginfo("Read aloud vib notification is set to False")
        else:
            rospy.loginfo("Read aloud vib notification is set to False (no CABOT_OPTION_READ_ALOUD_VIB_NOTIFICATION value)")

        i18n.set_language(self.lang)

        packages = ['cabot_ui']
        if self.site:
            packages.append(self.site)
        i18n.load_from_packages(packages)

    def speak(self, text, pose=None, force=True, pitch=50, volume=50, rate=50):
        if text is None:
            return
        try:
            rospy.wait_for_service('/speak', timeout=1)
        except rospy.ROSException as e:
            rospy.logerr(e)
            return

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


    def read_aloud_vibration(self, pattern=Handle.UNKNOWN):
        if not self.read_aloud:
            return

        if pattern == Handle.FRONT:
            self.speak(i18n.localized_string("HANDLE_START"))
        elif pattern == Handle.RIGHT_ABOUT_TURN:
            self.speak(i18n.localized_string("HANDLE_RIGHT_ABOUT_TURN"))
        elif pattern == Handle.RIGHT_TURN:
            self.speak(i18n.localized_string("HANDLE_RIGHT_TURN"))
        elif pattern == Handle.RIGHT_DEV:
            self.speak(i18n.localized_string("HANDLE_RIGHT_DEV"))
        elif pattern == Handle.LEFT_ABOUT_TURN:
            self.speak(i18n.localized_string("HANDLE_LEFT_ABOUT_TURN"))
        elif pattern == Handle.LEFT_TURN:
            self.speak(i18n.localized_string("HANDLE_LEFT_TURN"))
        elif pattern == Handle.LEFT_DEV:
            self.speak(i18n.localized_string("HANDLE_LEFT_DEV"))


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
        pass#self.speak(i18n.localized_string("CANCEL_NAVIGATION"))

    def resume_navigation(self):
        self.speak(i18n.localized_string("RESUME_NAVIGATION"))

    def start_exploration(self):
        pass#self.speak(i18n.localized_string("START_EXPLORATION"))


    ## navigate interface
    def i_am_ready(self):
        self.speak(i18n.localized_string("I_AM_READY"))

    def start_navigation(self, pose):
        self.vibrate(Handle.FRONT, pose=pose)
        self.read_aloud_vibration(Handle.FRONT)

    def notify_turn(self, turn=None, pose=None):
        if turn.angle < -math.pi/4*3:
            pattern = Handle.RIGHT_ABOUT_TURN
        elif turn.angle < -math.pi/3:
            pattern = Handle.RIGHT_TURN
        elif turn.angle < -math.pi/8:
            pattern = Handle.RIGHT_DEV
        elif turn.angle > math.pi/4*3:
            pattern = Handle.LEFT_ABOUT_TURN
        elif turn.angle > math.pi/3:
            pattern = Handle.LEFT_TURN
        elif turn.angle > math.pi/8:
            pattern = Handle.LEFT_DEV

        self.vibrate(pattern, pose=pose)
        self.read_aloud_vibration(pattern)
            
    def notify_human(self, angle=0, pose=None):
        vibration = Handle.RIGHT_DEV
        if angle > 0:
            vibration = Handle.LEFT_DEV

        self.vibrate(pattern=vibration, pose=pose)
        self.speak(i18n.localized_string("AVOIDING_A_PERSON"))

    def have_arrived(self, goal):
        name = goal.goal_name_pron
        desc = goal.goal_description

        if name:
            if desc:
                self.speak(i18n.localized_string("YOU_HAVE_ARRIVED_WITH_NAME_AND_DESCRIPTION").format(name,desc))
            else:
                self.speak(i18n.localized_string("YOU_HAVE_ARRIVED_WITH_NAME").format(name))
        else:
            self.speak(i18n.localized_string("YOU_HAVE_ARRIVED"))

    def approaching_to_avoiding_target(self, target=None, pose=None):
        statement = target.approaching_statement()
        if statement:
            self.speak(statement, pose)

    def approaching_to_poi(self, poi=None, pose=None):
        statement = poi.approaching_statement()
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

#    def request_action(self, goal=None, pose=None):
#        statement = goal.request_action_statement()
#        if statement:
#            self.speak(statement, pose)
#
#    def completed_action(self, goal=None, pose=None):
#        statement =goal.completed_action_statement()
#        if statement:
#            self.speak(statement, pose)
#
    def could_not_get_current_locaion(self):
        self.speak(i18n.localized_string("COULD_NOT_GET_CURRENT_LOCATION"))


    def enter_goal(self, goal):
        pass

    def exit_goal(self, goal):
        pass

    def announce_social(self, message):
        self.speak(i18n.localized_string(message))

    def please_call_elevator(self, pos):
        if pos:
            self.speak(i18n.localized_string("CALL_ELEVATOR_PLEASE_ON_YOUR",
                                             i18n.localized_string(pos)))
        else:
            self.speak(i18n.localized_string("CALL_ELEVATOR_PLEASE"))

    def elevator_opening(self, pose):
        self.vibrate(Handle.FRONT, pose=pose)
        self.speak(i18n.localized_string("ELEVATOR_IS_OPENING"))

    def floor_changed(self, floor):
        self.speak(i18n.localized_string("GETTING_OFF_THE_ELEVATOR"))

    def queue_start_arrived(self):
        self.speak(i18n.localized_string("GOING_TO_GET_IN_LINE"))

    def queue_proceed(self, pose=None):
        self.vibrate(Handle.FRONT, pose=pose)

    def please_pass_door(self):
        self.speak(i18n.localized_string("DOOR_POI_PASSED"))

    def door_passed(self):
        self.speak(i18n.localized_string("DOOR_POI_USER_ACTION"))
