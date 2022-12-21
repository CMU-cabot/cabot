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

from cmath import log
import os
import math

from rclpy.node import Node
from rclpy.exceptions import ROSInterruptException, InvalidServiceNameException
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from cabot_ui.cabot_rclpy_util import CaBotRclpyUtil

import std_msgs.msg
import cabot_msgs.msg
import cabot_msgs.srv
from cabot_ui import visualizer, geojson, i18n
from cabot_ui.turn_detector import Turn
from cabot_ui.stop_reasoner import StopReason
from cabot.handle_v2 import Handle

class UserInterface(object):
    SOCIAL_ANNOUNCE_INTERVAL = 15.0

    def __init__(self, node: Node):
        self._node = node
        self.visualizer = visualizer.instance(node)
        qos = QoSProfile(depth=10, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.note_pub = node.create_publisher(std_msgs.msg.Int8, "/cabot/notification", qos)
        self.activity_log_pub = node.create_publisher(cabot_msgs.msg.Log, "/cabot/activity_log", qos)
        self.pose_log_pub = node.create_publisher(cabot_msgs.msg.PoseLog, "/cabot/pose_log", qos)
        if not node.has_parameter("lang"):
            self.lang = node.declare_parameter("lang", "en").value
        else:
            self.lang = node.get_parameter("lang").value

        self.site = node.declare_parameter("site", '').value

        self.last_pose = None
        self.read_aloud = False
        if "CABOT_OPTION_READ_ALOUD_VIB_NOTIFICATION" in os.environ:
            if os.environ["CABOT_OPTION_READ_ALOUD_VIB_NOTIFICATION"] == "1":
                self.read_aloud = True
                CaBotRclpyUtil.info("Read aloud vib notification is set to True")
            else:
                CaBotRclpyUtil.info("Read aloud vib notification is set to False")
        else:
            CaBotRclpyUtil.info("Read aloud vib notification is set to False (no CABOT_OPTION_READ_ALOUD_VIB_NOTIFICATION value)")

        i18n.set_language(self.lang)

        packages = ['cabot_ui']
        if self.site:
            packages.append(self.site)
        i18n.load_from_packages(packages)

        self.last_social_announce = None

    def _activity_log(self, category="", text="", memo="", visualize=False):
        log = cabot_msgs.msg.Log()
        log.header.stamp = self._node.get_clock().now().to_msg()
        log.category = category
        log.text = text
        log.memo = memo
        self.activity_log_pub.publish(log)
        CaBotRclpyUtil.info(F"{category}:{text}:{memo}")

        if visualize and self.last_pose is not None:
            self.visualizer.spoken.append((self.last_pose['ros_pose'], "%s:%s".format(text, memo), category))
            self.visualizer.visualize()

    def _pose_log(self):
        if not self.last_pose:
            return
        log = cabot_msgs.msg.PoseLog()
        log.header.stamp = self._node.get_clock().now().to_msg()
        log.header.frame_id = self.last_pose['global_frame']
        log.pose = self.last_pose['ros_pose']
        log.lat = self.last_pose['global_position'].lat
        log.lng = self.last_pose['global_position'].lng
        log.floor = self.last_pose['current_floor']
        self.pose_log_pub.publish(log)

    def speak(self, text, force=True, pitch=50, volume=50, rate=50):
        if text is None:
            return
        
        self._activity_log("speech request", text, self.lang, visualize=True)
        
        cli = self._node.create_client(cabot_msgs.srv.Speak)
        try:
            cli.wait_for_service(timeout_sec=1.0)
        except InvalidServiceNameException as e:
            CaBotRclpyUtil.error(e)
            return

        # tts.speak(text, force=force, pitch=pitch, volume=volume, rate=rate, lang=self.lang)
        try:
            rate = self._node.get_parameter("/cabot/cabot_menu_node/speech_speed/value").value
        except KeyError:
            CaBotRclpyUtil.info("could not get param '/cabot/cabot_menu_node/speech_speed/value'")

        voice = "female"
        try:
            voice = self._node.get_parameter("/cabot/cabot_menu_node/speech_voice/value")
            CaBotRclpyUtil.info(F"voice {voice}")
            if voice != "male" or voice != "female":
                voice = "female"
        except KeyError:
            CaBotRclpyUtil.info("could not get param '/cabot/cabot_menu_node/speech_voice/value'")

        speak_proxy = self._node.create_client(cabot_msgs.srv.Speak, '/speak')
        try:
            speak_proxy.wait_for_service(timeout_sec=2)
            CaBotRclpyUtil.info(F"try to speak {text} (v={voice}, r={rate}, p={pitch}) {force}", text.encode("utf-8"))
            speak_proxy(text, rate, pitch, volume, self.lang, voice, force, 50, 2, cabot_msgs.srv.SpeakRequest.CHANNEL_BOTH)
            CaBotRclpyUtil.info("speak finished")
        except InvalidServiceNameException as e:
            CaBotRclpyUtil.Error(F"Service call failed: {e}")


    def vibrate(self, pattern=Handle.UNKNOWN):
        self._activity_log("cabot/interface", "vibration", Handle.get_name(pattern), visualize=True)
        msg = std_msgs.msg.Int8()
        msg.data = pattern
        self.note_pub.publish(msg)

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
        self._activity_log("cabot/interface", "navigation", "pause")
        self.speak(i18n.localized_string("PAUSE_NAVIGATION"))

    def cancel_navigation(self):
        pass#self.speak(i18n.localized_string("CANCEL_NAVIGATION"))

    def resume_navigation(self):
        self._activity_log("cabot/interface", "navigation", "resume")
        self.speak(i18n.localized_string("RESUME_NAVIGATION"))

    def start_exploration(self):
        pass#self.speak(i18n.localized_string("START_EXPLORATION"))


    ## navigate interface
    def activity_log(self, category="", text="", memo=""):
        self._activity_log(category, text, memo)

    def i_am_ready(self):
        self._activity_log("cabot/interface", "status", "ready")
        self.speak(i18n.localized_string("I_AM_READY"))

    def start_navigation(self):
        self._activity_log("cabot/interface", "navigation", "start")
        self.vibrate(Handle.FRONT)
        self.read_aloud_vibration(Handle.FRONT)

    def update_pose(self, **kwargs):
        self.last_pose = kwargs
        self._pose_log()

    def notify_turn(self, turn=None, pose=None):
        pattern = Handle.UNKNOWN
        text = ""
        if turn.turn_type == Turn.Type.Normal:
            if turn.angle < -math.pi/4*3:
                pattern = Handle.RIGHT_ABOUT_TURN
                text = "right about turn"
            elif turn.angle < -math.pi/3:
                pattern = Handle.RIGHT_TURN
                text = "right turn"
            elif turn.angle > math.pi/4*3:
                pattern = Handle.LEFT_ABOUT_TURN
                text = "left about turn"
            elif turn.angle > math.pi/3:
                pattern = Handle.LEFT_TURN
                text = "left turn"
        elif turn.turn_type == Turn.Type.Avoiding:
            if turn.angle < 0:
                pattern = Handle.RIGHT_DEV
                text = "slight right"
            if turn.angle > 0:
                pattern = Handle.LEFT_DEV
                text = "slight left"

        self._activity_log("cabot/interface", "turn angle", str(turn.angle))
        self._activity_log("cabot/interface", "notify", text)
        self.vibrate(pattern)
        self.read_aloud_vibration(pattern)
            
    def notify_human(self, angle=0):
        vibration = Handle.RIGHT_DEV
        if angle > 0:
            vibration = Handle.LEFT_DEV

        self._activity_log("cabot/interface", "human")
        self.vibrate(pattern=vibration)
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
        self._activity_log("cabot/interface", "navigation", "arrived")

    def approaching_to_poi(self, poi=None):
        statement = poi.approaching_statement()
        if statement:
            self.speak(statement)
            self._activity_log("cabot/interface", "poi", "approaching")

    def approached_to_poi(self, poi=None):
        statement = poi.approached_statement()
        if statement:
            self.speak(statement)
            self._activity_log("cabot/interface", "poi", "approached")

    def passed_poi(self, poi=None):
        statement = poi.passed_statement()
        if statement:
            self.speak(statement)
            self._activity_log("cabot/interface", "poi", "passed")

    def could_not_get_current_location(self):
        self.speak(i18n.localized_string("COULD_NOT_GET_CURRENT_LOCATION"))

    def enter_goal(self, goal):
        pass

    def exit_goal(self, goal):
        pass

    def announce_social(self, message):
        self._activity_log("cabot/interface", "notify", "social")
        if self.last_social_announce is None or \
            (self._node.get_clock().now() - self.last_social_announce).to_sec() > UserInterface.SOCIAL_ANNOUNCE_INTERVAL:
            self.speak(i18n.localized_string(message))
            self.last_social_announce = self._node.get_clock().now()

    def please_call_elevator(self, pos):
        self._activity_log("cabot/interface", "navigation", "elevator button")
        if pos:
            self.speak(i18n.localized_string("CALL_ELEVATOR_PLEASE_ON_YOUR",
                                             i18n.localized_string(pos)))
        else:
            self.speak(i18n.localized_string("CALL_ELEVATOR_PLEASE"))

    def elevator_opening(self):
        self._activity_log("cabot/interface", "navigation", "elevator opening")
        self.vibrate(Handle.FRONT)
        self.speak(i18n.localized_string("ELEVATOR_IS_OPENING"))

    def floor_changed(self, floor):
        self._activity_log("cabot/interface", "navigation", "floor_changed")
        self.speak(i18n.localized_string("GETTING_OFF_THE_ELEVATOR"))

    def queue_start_arrived(self):
        self._activity_log("cabot/interface", "queue", "start arrived")
        self.speak(i18n.localized_string("GOING_TO_GET_IN_LINE"))

    def queue_proceed(self):
        self._activity_log("cabot/interface", "queue", "proceed")
        self.vibrate(Handle.FRONT)

    def please_pass_door(self):
        self._activity_log("cabot/interface", "navigation", "manual door")
        self.speak(i18n.localized_string("DOOR_POI_USER_ACTION"))

    def door_passed(self):
        self._activity_log("cabot/interface", "navigation", "door passed")
        self.speak(i18n.localized_string("DOOR_POI_PASSED"))

    def speak_stop_reason(self, code):
        message = None
        if code == StopReason.AVOIDING_PEOPLE:
            message = "TRYING_TO_AVOID_PEOPLE"
        elif code == StopReason.AVOIDING_OBSTACLE:
            message = "TRYING_TO_AVOID_OBSTACLE"
        elif code == StopReason.THERE_ARE_PEOPLE_ON_THE_PATH:
            message = "PEOPLE_ARE_ON_MY_WAY"
        elif code == StopReason.UNKNOWN:
            message = "PLEASE_WAIT_FOR_A_SECOND"
        if message:
            self.announce_social(message)

    def please_follow_behind(self):
        self.speak(i18n.localized_string("FOLLOW_BEHIND_PLEASE_NARROW"))

    def please_return_position(self):
        self.speak(i18n.localized_string("RETURN_TO_POSITION_PLEASE"))
