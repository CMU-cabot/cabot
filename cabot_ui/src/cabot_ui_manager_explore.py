#!/usr/bin/env python

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
"""
Cabot UI Manager

This class manages the state of the robot. 
It serves low level event and maps to high level event which may change state of the robot.

Ideally, this class has plugin architecture to add new UI component but it is not the current goal.
So, all controls which needs to see the current state of the robot are managed by this code.

Low-level (cabot.event) should be mapped into ui-level (cabot_ui.event)

Author: Daisuke Sato<daisuke@cmu.edu>
"""

import traceback
from cabot.util import setInterval
import rospy
import std_msgs.msg
import geometry_msgs.msg
import std_srvs.srv
import nav_msgs.msg
import math
import time

import cabot
import cabot.button
from cabot.event import BaseEvent, ButtonEvent, ClickEvent
from cabot_ui.event import NavigationEvent
from cabot_ui.menu import Menu
from cabot_ui.navigation_explore import NavigationExplore
from cabot_ui.status_explore import StateExplore, StatusManagerExplore
from cabot_ui.interface_explore import UserInterfaceExplore


class CabotUIManagerExplore(object):
    def __init__(self):
        self.main_menu = Menu.create_menu({"menu":"main_menu"}, name_space=rospy.get_name())

        self.speed_menu = None
        if self.main_menu:
            self.main_menu.delegate = self
            self.speed_menu = self.main_menu.get_menu_by_identifier("max_velocity_menu")
        else:
            rospy.logerr("menu is not initialized")
            
        if self.speed_menu:
            init_speed = self.speed_menu.value
            try:
                init_speed = float(rospy.get_param("~init_speed", self.speed_menu.value))
            except ValueError:
                pass
 
            rospy.logdebug("Initial Speed = %.2f", init_speed)
            self.speed_menu.set_value(init_speed)
    

        self.menu_stack = []

        self.reset()

        self._event_mapper = ExploreEventMapper()
        self._event_mapper.delegate = self
        self._status_manager = StatusManagerExplore.get_instance()
        self._status_manager.delegate = self
        self._interface = UserInterfaceExplore()
        self._interface.delegate = self
        self._navigation = NavigationExplore()
        self._navigation.delegate = self

        self._retry_count = 0
        self._linear_x = 0.1
        self._angular_z = math.pi * 0.1
        self._pause_control = False

        rospy.Subscriber("/cabot/event", std_msgs.msg.String,
                         self._event_callback, None)
        self._eventPub = rospy.Publisher("/cabot/event", std_msgs.msg.String, queue_size=1)

        self._cabot_explore_cmd_vel_pub = rospy.Publisher("/cmd_vel", geometry_msgs.msg.Twist, queue_size=1)
        self._cabot_explore_pause_control_pub = rospy.Publisher("/cabot/pause_control", std_msgs.msg.Bool, queue_size=1)

        rospy.wait_for_service("set_touch_speed_active_mode")
        self._touchModeProxy = rospy.ServiceProxy("set_touch_speed_active_mode", std_srvs.srv.SetBool)

        rospy.wait_for_service("/cabot/user_speed_enabled")
        self._userSpeedEnabledProxy = rospy.ServiceProxy("/cabot/user_speed_enabled", std_srvs.srv.SetBool)

        ## for sign reco
        self._odomSub = rospy.Subscriber("/odom", nav_msgs.msg.Odometry, self._odom_callback)
        self._last_odom = None

        try:
            rospy.wait_for_service("/sign_reco", timeout=1)
            self._sign_reco_proxy = rospy.ServiceProxy("/sign_reco", std_srvs.srv.Trigger)
        except:
            rospy.logerr("could not find sign_reco service")
            self._sign_reco_proxy = None

        self._status_manager.set_state(StateExplore.processing)
        
        rospy.sleep(3)
        self._navigation.observe_once()
        
        self._status_manager.set_state(StateExplore.decision_making)

        self.publish_to_pause_control()

        self.report_current_status()

    def _odom_callback(self, msg):
        self._last_odom = msg

    ### navigation delegate
    def i_am_ready(self):
        self._interface.i_am_ready()

    def start_explore(self):
        self._interface.start_explore()
        self._status_manager.set_state(StateExplore.moving_next_intersection)

    def start_explore_forward(self):
        self._interface.start_explore_forward()
        self._status_manager.set_state(StateExplore.moving_next_corner)

    def finish_explore_forward(self):
        self._interface.finish_explore_forward()

    def go_origin(self):
        self._interface.go_origin()
        self._status_manager.set_state(StateExplore.moving_origin)

    def arrive_origin(self):
        self._interface.arrive_origin()
        self._status_manager.set_state(StateExplore.decision_making)

    def system_cancel_explore(self):
        self._interface.system_cancel_explore()
        self._status_manager.set_state(StateExplore.decision_making)

    def user_cancel_explore(self):
        self._interface.user_cancel_explore()
        self._status_manager.set_state(StateExplore.decision_making)

    def notify_turn(self, turn=None, pose=None):
        self._interface.notify_turn(turn=turn, pose=pose)

    def find_forward_right_left_route(self):
        self._interface.find_forward_right_left_route()
        self._status_manager.set_state(StateExplore.decision_making)

    def find_forward_right_route(self):
        self._interface.find_forward_right_route()
        self._status_manager.set_state(StateExplore.decision_making)

    def find_forward_left_route(self):
        self._interface.find_forward_left_route()
        self._status_manager.set_state(StateExplore.decision_making)

    def find_right_left_route(self):
        self._interface.find_right_left_route()
        self._status_manager.set_state(StateExplore.decision_making)

    def find_right_route(self):
        self._interface.find_right_route()
        self._status_manager.set_state(StateExplore.decision_making)

    def find_left_route(self):
        self._interface.find_left_route()
        self._status_manager.set_state(StateExplore.decision_making)

    def find_dead_end(self):
        self._interface.find_dead_end()
        self._status_manager.set_state(StateExplore.decision_making)

    def find_intersection_routes_in_clock_direction(self,paths):
        self._interface.find_intersection_routes_in_clock_direction(paths)
        self._status_manager.set_state(StateExplore.decision_making)

    def error(self):
        self._status_manager.set_state(StateExplore.decision_making)

    def observe_once(self):
        self._status_manager.set_state(StateExplore.decision_making)

    def turn_right_route(self):
        self._interface.turn_right_route()

    def turn_left_route(self):
        self._interface.turn_left_route()

    def cannot_move_forward_route(self):
        self._interface.cannot_move_forward_route()

    def cannot_turn_right_route(self):
        self._interface.cannot_turn_right_route()

    def cannot_turn_left_route(self):
        self._interface.cannot_turn_left_route()

    def could_not_get_current_locaion(self):
        self._interface.could_not_get_current_locaion()

    def cannot_turn_route(self):
        self._interface.cannot_turn_route()

    ###
    def _turn_right_done_callback(self, status, result):
        rospy.loginfo("turn_right_done_callback")
        self._navigation.finish_turn("right")
        self._status_manager.set_state(StateExplore.decision_making)

    def _turn_left_done_callback(self, status, result):
        rospy.loginfo("turn_left_done_callback")
        self._navigation.finish_turn("left")
        self._status_manager.set_state(StateExplore.decision_making)

    def _event_callback(self, msg):
        event = BaseEvent.parse(msg.data)
        if event is None:
            rospy.logerr("cabot event %s cannot be parsed", msg.data)
            return
        self.process_event(event)

    def reset(self):
        """reset menu"""
        if self.main_menu:
            self.main_menu.reset()
        self.menu_stack = [self.main_menu]

    # menu delegate method
    def menu_selected(self, menu):
        rospy.logdebug("menu_selected, %s, %s"%(menu.identifier, menu.type))

    # event delegate method
    def process_event(self, event):
        '''
        all events go through this method
        '''
        rospy.loginfo("process_event %s", str(event))

        self._event_mapper.push(event)
        self._process_navigation_event(event)

    def _process_navigation_event(self, event):
        if event.type != NavigationEvent.TYPE:
            return

        if event.subtype == "speedup":
            self.speed_menu.prev()
            self._interface.menu_changed(menu=self.speed_menu)
            e = NavigationEvent("sound", "SpeedUp")
            msg = std_msgs.msg.String()
            msg.data = str(e)
            self._eventPub.publish(msg)

        if event.subtype == "speeddown":
            self.speed_menu.next()
            self._interface.menu_changed(menu=self.speed_menu)
            e = NavigationEvent("sound", "SpeedDown")
            msg = std_msgs.msg.String()
            msg.data = str(e)
            self._eventPub.publish(msg)

        if event.subtype == "start":
            rospy.loginfo("start explore")
            self.start_explore()
            self._navigation.start_explore()

        if event.subtype == "start_forward":
            rospy.loginfo("start explore forward")
            self.start_explore_forward()
            self._navigation.start_explore_forward()

        if event.subtype == "go_origin":
            rospy.loginfo("start go origin")
            self.go_origin()
            self._navigation.go_origin()

        if event.subtype == "stop":
            self._navigation.user_cancel_explore()
            self.user_cancel_explore()

        if event.subtype == "stop_and_observe_once":
            self._status_manager.set_state(StateExplore.processing)
            self._navigation.user_cancel_explore()
            self._interface.user_cancel_explore()
            while self._last_odom is None or abs(self._last_odom.twist.twist.linear.x) > 0.01: pass
            self._navigation.observe_once()
            self.observe_once()

        if event.subtype == "turn_right":
            self._status_manager.set_state(StateExplore.processing)
            # self.turn_right_route()
            self._navigation.turn_right_route(self._turn_right_done_callback)

        if event.subtype == "turn_left":
            self._status_manager.set_state(StateExplore.processing)
            # self.turn_left_route()
            self._navigation.turn_left_route(self._turn_left_done_callback)

        if event.subtype == "observe_intersection":
            self._status_manager.set_state(StateExplore.processing)
            self._navigation.observe_once()
            self.observe_once()

        if event.subtype == "turn_mortor":
            self._pause_control = not self._pause_control

        if event.subtype == "sign_reco":
            '''
            todo
            receive completion handler (of image process) from iphone and when that is done, set the state to decision making
            '''
            self._status_manager.set_state(StateExplore.processing)
            
            if self._sign_reco_proxy is None:
                try:
                    rospy.wait_for_service("/sign_reco", timeout=1)
                    self._sign_reco_proxy = rospy.ServiceProxy("/sign_reco", std_srvs.srv.Trigger)
                except:
                    rospy.logerr("could not find sign_reco service")

            if self._sign_reco_proxy is not None:
                self._sign_reco_proxy()
                rospy.sleep(3)
            else:
                rospy.logerr("sign_reco service is not available")
            self._status_manager.set_state(StateExplore.decision_making)

        if event.subtype == "stop_and_sign_reco":
            self._status_manager.set_state(StateExplore.processing)

            self._navigation.user_cancel_explore()
            while abs(self._last_odom.twist.twist.linear.x) > 0.01: pass

            if self._sign_reco_proxy is None:
                try:
                    rospy.wait_for_service("/sign_reco", timeout=1)
                    self._sign_reco_proxy = rospy.ServiceProxy("/sign_reco", std_srvs.srv.Trigger)
                except:
                    rospy.logerr("could not find sign_reco service")

            if self._sign_reco_proxy is not None:
                self._sign_reco_proxy()
                rospy.sleep(3)
            else:
                rospy.logerr("sign_reco service is not available")
            self._status_manager.set_state(StateExplore.decision_making)

        if event.subtype == "forward_override_and_observe_once":
            self._interface.finding_intersection()
            self._status_manager.set_state(StateExplore.processing)
            twist = geometry_msgs.msg.Twist()
            twist.linear.x = 0.1
            start = time.time()
            elapsed = 0.0
            while elapsed < 5.0:
                self._cabot_explore_cmd_vel_pub.publish(twist)
                elapsed = time.time() - start
                rospy.loginfo("sleeping for override " + str(elapsed))
            twist.linear.x = 0.0
            self._cabot_explore_cmd_vel_pub.publish(twist)
            self._navigation.user_cancel_explore()
            self._navigation.observe_once()
            self._status_manager.set_state(StateExplore.decision_making)

        if event.subtype == "event":
            self._navigation.process_event(event)

    @setInterval(1.0)
    def publish_to_pause_control(self):
        pause_control = std_msgs.msg.Bool()
        pause_control.data = self._pause_control
        self._cabot_explore_pause_control_pub.publish(pause_control)

    @setInterval(0.1)
    def report_current_status(self):
        rospy.logdebug("current status is: " + str(self._status_manager._current))

class ExploreEventMapper(object):
    def __init__(self):
        self._manager = StatusManagerExplore.get_instance()
        self._disableClick = False
    
    def push(self, event):
        state = self._manager.state

        if event.type != ButtonEvent.TYPE and event.type != ClickEvent.TYPE:
            return

        mevent = None

        # simplify the control
        mevent = self.map_button_to_navigation(event)

        if mevent:
            self.delegate.process_event(mevent)

    def map_button_to_navigation(self, event):
        rospy.loginfo("map_button_to_navigation, self._manager.state = " + str(self._manager.state))

        if event.type==ClickEvent.TYPE:
            if self._disableClick:
                self._disableClick = False
                return None
        elif event.type==ButtonEvent.TYPE:
            if event.hold:
                self._disableClick = True
                if self._disableHold:
                    return None
                self._disableHold = True
            elif event.up:
                self._disableHold = False

        if self._manager.state==StateExplore.decision_making:
            
            if event.type==ClickEvent.TYPE:
                if event.buttons == cabot.button.BUTTON_UP:
                    rospy.loginfo("start")
                    return NavigationEvent(subtype="start")
                elif event.buttons == cabot.button.BUTTON_RIGHT:
                    rospy.loginfo("turn_right")
                    return NavigationEvent(subtype="turn_right")
                elif event.buttons == cabot.button.BUTTON_LEFT:
                    rospy.loginfo("turn_left")
                    return NavigationEvent(subtype="turn_left")
                elif event.buttons == cabot.button.BUTTON_DOWN:
                    if event.count == 1:
                        rospy.loginfo("sign_reco")
                        return NavigationEvent(subtype="sign_reco")
                    elif event.count == 3:
                        rospy.loginfo("go_origin")
                        return NavigationEvent(subtype="go_origin")
            elif event.type==ButtonEvent.TYPE and event.hold:
                if event.button == cabot.button.BUTTON_UP:
                    rospy.loginfo("forward override and observe once")
                    return NavigationEvent(subtype="forward_override_and_observe_once")
                elif event.button == cabot.button.BUTTON_DOWN:
                    rospy.loginfo("turn mortor")
                    return NavigationEvent(subtype="turn_mortor")

        elif self._manager.state==StateExplore.moving_next_intersection or self._manager.state==StateExplore.moving_next_corner or self._manager.state==StateExplore.moving_origin:
        
            if event.type==ClickEvent.TYPE:
                if event.buttons == cabot.button.BUTTON_UP:
                    if self._manager.state==StateExplore.moving_next_intersection:
                        rospy.loginfo("start forward")
                        return NavigationEvent(subtype="start_forward")
                    elif self._manager.state==StateExplore.moving_next_corner:
                        rospy.loginfo("start")
                        return NavigationEvent(subtype="start")
                elif event.buttons == cabot.button.BUTTON_DOWN:
                    rospy.loginfo("stop and sign reco")
                    return NavigationEvent(subtype="stop_and_sign_reco")

        elif self._manager.state==StateExplore.processing:
            return None

        return None

if __name__ == "__main__":
    rospy.init_node("cabot_ui_manager_explore", log_level=rospy.DEBUG)
    try:
        CabotUIManagerExplore()
    except:
        rospy.logerr(traceback.format_exc())
    rospy.spin()
