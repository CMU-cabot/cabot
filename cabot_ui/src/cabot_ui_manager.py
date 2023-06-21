#!/usr/bin/env python

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
Cabot UI Manager

This class manages the state of the robot. 
It serves low level event and maps to high level event which may change state of the robot.

Ideally, this class has plugin architecture to add new UI component but it is not the current goal.
So, all controls which needs to see the current state of the robot are managed by this code.

Low-level (cabot.event) should be mapped into ui-level (cabot_ui.event)

Author: Daisuke Sato<daisuke@cmu.edu>
"""

import traceback
import rospy
import std_msgs.msg
import std_srvs.srv

import cabot
from cabot import util
import cabot.button
from cabot.event import BaseEvent, ButtonEvent, ClickEvent, HoldDownEvent, JoyButtonEvent, JoyClickEvent
from cabot_ui.event import MenuEvent, NavigationEvent, ExplorationEvent
from cabot_ui.menu import Menu
from cabot_ui.status import State, StatusManager
from cabot_ui.interface import UserInterface
from cabot_ui.navigation import Navigation, NavigationInterface
from cabot_ui.exploration import Exploration
from cabot_ui.stop_reasoner import StopReason
from diagnostic_updater import Updater, FunctionDiagnosticTask
from diagnostic_msgs.msg import DiagnosticStatus


class CabotUIManager(NavigationInterface, object):
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
        self.in_navigation = False
        self.destination = None

        self.reset()

        self._event_mapper = EventMapper()
        self._event_mapper.delegate = self
        self._status_manager = StatusManager.get_instance()
        self._status_manager.delegate = self
        self._interface = UserInterface()
        self._interface.delegate = self
        self._navigation = Navigation()
        self._navigation.delegate = self
        self._exploration = Exploration()
        self._exploration.delegate = self

        self._retry_count = 0

        rospy.Subscriber("/cabot/event", std_msgs.msg.String,
                         self._event_callback, None)
        self._eventPub = rospy.Publisher("/cabot/event", std_msgs.msg.String, queue_size=1)

        rospy.wait_for_service("set_touch_speed_active_mode")
        self._touchModeProxy = rospy.ServiceProxy("set_touch_speed_active_mode", std_srvs.srv.SetBool)

        rospy.wait_for_service("/cabot/user_speed_enabled")
        self._userSpeedEnabledProxy = rospy.ServiceProxy("/cabot/user_speed_enabled", std_srvs.srv.SetBool)

        self.updater = Updater()
        rospy.Timer(rospy.Duration(1), lambda e: self.updater.update())
        def manager_status(stat):
            if self._navigation.i_am_ready:
                stat.summary(DiagnosticStatus.OK, "Ready")
            else:
                stat.summary(DiagnosticStatus.ERROR, "Not ready")
        self.updater.add(FunctionDiagnosticTask("UI Manager", manager_status))

    ### navigation delegate
    def activity_log(self, category="", text="", memo=""):
        self._interface.activity_log(category, text, memo)

    def i_am_ready(self):
        self._interface.i_am_ready()

    def start_navigation(self):
        rospy.loginfo("self._interface.start_navigation()")
        self._interface.start_navigation()

    def update_pose(self, **kwargs):
        self._interface.update_pose(**kwargs)

    def notify_turn(self, turn=None):
        self._interface.notify_turn(turn=turn)

    def notify_human(self, angle=0):
        self._interface.notify_human(angle=angle)

    def goal_canceled(self, goal):
        # unexpected cancel, may need to retry
        if self._status_manager.state == State.in_action:
            rospy.loginfo("NavigationState: canceled (system)")
            self._status_manager.set_state(State.in_pausing)
            self._retry_navigation()
            return
        rospy.loginfo("NavigationState: canceled (user)")

    @util.setInterval(2, times=1)
    def _retry_navigation(self):
        self._retry_count += 1
        rospy.loginfo("NavigationState: retrying (system)")
        self._navigation.retry_navigation()
        self._status_manager.set_state(State.in_action)
        rospy.loginfo("NavigationState: retried (system)")

    def have_arrived(self, goal):
        #rospy.loginfo("delegate have_arrived called")
        #self._interface.have_arrived(goal)
        rospy.loginfo("NavigationState: arrived")

        # notify external nodes about arrival
        e = NavigationEvent("arrived", None)
        msg = std_msgs.msg.String()
        msg.data = str(e)
        self._eventPub.publish(msg)

    def approaching_to_poi(self, poi=None):
        self._interface.approaching_to_poi(poi=poi)

    def approached_to_poi(self, poi=None):
        self._interface.approached_to_poi(poi=poi)

    def passed_poi(self, poi=None):
        self._interface.passed_poi(poi=poi)

    def could_not_get_current_location(self):
        self._interface.could_not_get_current_location()

    def enter_goal(self, goal):
        self._interface.enter_goal(goal)

    def exit_goal(self, goal):
        self._interface.exit_goal(goal)

    def announce_social(self, message):
        self._interface.announce_social(message)

    def please_call_elevator(self, pos):
        self._interface.please_call_elevator(pos)

    def elevator_opening(self):
        self._interface.elevator_opening()

    def floor_changed(self, floor):
        self._interface.floor_changed(floor)

    def queue_start_arrived(self):
        self._interface.queue_start_arrived()

    def queue_proceed(self):
        self._interface.queue_proceed()

    def queue_target_arrived(self):
        self._interface.queue_target_arrived()

    def please_pass_door(self):
        self._interface.please_pass_door()

    def door_passed(self):
        self._interface.door_passed()

    def please_follow_behind(self):
        self._interface.please_follow_behind()

    def please_return_position(self):
        self._interface.please_return_position()

    ###
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
        if menu.identifier == "destination_menu":
            event = NavigationEvent("destination", menu.value.value)
            self.process_event(event)

        if menu.identifier == "main_menu" and menu.value is not None:
            rospy.loginfo(menu.value)
            rospy.loginfo(menu.value.identifier)
            if menu.value.identifier == "exploration_menu":
                event = ExplorationEvent("start")
                self.process_event(event)


    # event delegate method
    def process_event(self, event):
        '''
        all events go through this method
        '''
        #rospy.loginfo("process_event %s", str(event))

        self._event_mapper.push(event)
        self._process_menu_event(event)
        self._process_navigation_event(event)
        self._process_exploration_event(event)


    def _process_menu_event(self, event):
        '''
        process only menu event
        '''
        if event.type != MenuEvent.TYPE:
            return
        
        curr_menu = self.menu_stack[-1]
        if event.subtype == "next":
            curr_menu.next()
            self._interface.menu_changed(menu=curr_menu)

        elif event.subtype == "prev":
            curr_menu.prev()
            self._interface.menu_changed(menu=curr_menu)

        elif event.subtype == "select":
            selected = curr_menu.select()
            if selected is None: ## from main menu
                if curr_menu.value is None:
                    curr_menu.next()
                selected = curr_menu
            elif not selected.can_explore:
                self.reset()
            elif selected is not curr_menu:
                self.menu_stack.append(selected)
                if selected.value is None:
                    selected.next()

            self._interface.menu_changed(menu=selected, usage=True)

        elif event.subtype == "back":
            if len(self.menu_stack) > 1:
                self.menu_stack.pop()
                curr_menu = self.menu_stack[-1]

            self._interface.menu_changed(menu=curr_menu, backed=True) 
            
            self.speed = 0
            #self.cancel_pub.publish(True)
            self._navigation.pause_navigation()


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


        if event.subtype == "destination":
            rospy.loginfo("Destination: "+event.param)
            self._retry_count = 0
            self._navigation.set_destination(event.param)
            self.destination = event.param
            ## change handle mode
            try:
                self._touchModeProxy(True)
            except rospy.ServiceException as e:
                rospy.loginfo("Could not set touch mode to True")
            try:
                self._userSpeedEnabledProxy(True)
            except rospy.ServiceException as e:
                rospy.loginfo("Could not set user speed enabled to True")

            ## change state
            # change to waiting_action by using actionlib
            self._status_manager.set_state(State.in_action)

        if event.subtype == "summons":
            rospy.loginfo("Summons Destination: "+event.param)
            self._navigation.set_destination(event.param)
            self.destination = event.param
            ## change handle mode
            try:
                self._touchModeProxy(False)
            except rospy.ServiceException as e:
                rospy.loginfo("Could not set touch mode to False")
            try:
                self._userSpeedEnabledProxy(False)
            except rospy.ServiceException as e:
                rospy.loginfo("Could not set user speed enabled to False")

            ## change state
            # change to waiting_action by using actionlib
            self._status_manager.set_state(State.in_summons)

        if event.subtype == "event":
            self._navigation.process_event(event)

        if event.subtype == "cancel":
            rospy.loginfo("NavigationState: User Cancel requested")
            if self._status_manager.state == State.in_action or \
               self._status_manager.state == State.in_summons:
                rospy.loginfo("NavigationState: canceling (user)")
                self._interface.cancel_navigation()
                self._navigation.cancel_navigation()
                self.in_navigation = False
                self.destination = None
                self._status_manager.set_state(State.idle)
                rospy.loginfo("NavigationState: canceled (user)")
            else:
                rospy.loginfo("NavigationState: state is not in action state={}".format(self._status_manager.state))

        if event.subtype == "pause":
            rospy.loginfo("NavigationState: User Pause requested")
            if self._status_manager.state == State.in_action or \
               self._status_manager.state == State.in_summons:
                rospy.loginfo("NavigationState: pausing (user)")
                self._status_manager.set_state(State.in_pausing)
                self._interface.pause_navigation()
                self._navigation.pause_navigation()
                self._status_manager.set_state(State.in_pause)
                rospy.loginfo("NavigationState: paused (user)")
            else:
                # force to pause state
                rospy.loginfo("NavigationState: state is not in action state={}".format(self._status_manager.state))
                #self._status_manager.set_state(State.in_pausing)
                #self._navigation.pause_navigation()
                #self._status_manager.set_state(State.in_pause)

        if event.subtype == "resume":
            if self.destination is not None:
                rospy.loginfo("NavigationState: User Resume requested")
                if self._status_manager.state == State.in_pause:
                    rospy.loginfo("NavigationState: resuming (user)")
                    self._interface.resume_navigation()
                    self._navigation.resume_navigation()
                    self._status_manager.set_state(State.in_action)
                    rospy.loginfo("NavigationState: resumed (user)")
                else:
                    rospy.loginfo("NavigationState: state is not in pause state")
            else:
                rospy.loginfo("NavigationState: Next")
                e = NavigationEvent("next", None)
                msg = std_msgs.msg.String()
                msg.data = str(e)
                self._eventPub.publish(msg)

            # activate control
            rospy.loginfo("NavigationState: Pause control = False")
            self._interface.set_pause_control(False)
            self._navigation.set_pause_control(False)

        if event.subtype == "decision":
            if self.destination is None:
                rospy.loginfo("NavigationState: Subtour")
                e = NavigationEvent("subtour", None)
                msg = std_msgs.msg.String()
                msg.data = str(e)
                self._eventPub.publish(msg)

        if event.subtype == "arrived":
            self.destination = None

        if event.subtype == "stop-reason":
            code = StopReason[event.param]
            self._interface.speak_stop_reason(code)

        # deactivate control
        if event.subtype == "idle":
            rospy.loginfo("NavigationState: Pause control = True")
            self._interface.set_pause_control(True)
            self._navigation.set_pause_control(True)


    def _process_exploration_event(self, event):
        if event.type != ExplorationEvent.TYPE:
            return


        if event.subtype == "start":
            self._interface.start_exploration()
            self._exploration.start_exploration()


class EventMapper(object):
    def __init__(self):
        self._manager = StatusManager.get_instance()
    
    def push(self, event):
        state = self._manager.state

        if event.type != ButtonEvent.TYPE and event.type != ClickEvent.TYPE and \
            event.type != HoldDownEvent.TYPE:
            return

        mevent = None

        # simplify the control
        mevent = self.map_button_to_navigation(event)

        '''
        if state == State.idle:
            mevent = self.map_button_to_menu(event)
            
        elif state == State.in_action or state == State.waiting_action:
            mevent = self.map_button_to_navigation(event)
            
        elif state == State.in_pause or state == State.waiting_pause:
            mevent = self.map_button_to_navigation(event)
        '''

        if mevent:
            self.delegate.process_event(mevent)


    def map_button_to_menu(self, event):
        if event.type == "click" and event.count == 1:
            if event.buttons == cabot.button.BUTTON_NEXT:
                return MenuEvent(subtype="next")
            if event.buttons == cabot.button.BUTTON_PREV:
                return MenuEvent(subtype="prev")
            if event.buttons == cabot.button.BUTTON_SELECT:
                return MenuEvent(subtype="select")
        elif event.type == "click" and event.count == 2:
            if event.buttons == cabot.button.BUTTON_SELECT:
                return MenuEvent(subtype="back")
        return None

    def map_button_to_navigation(self, event):
        if event.type == "button" and event.down:
            if event.button == cabot.button.BUTTON_UP:
                return NavigationEvent(subtype="speedup")
            if event.button == cabot.button.BUTTON_DOWN:
                return NavigationEvent(subtype="speeddown")
            if event.button == cabot.button.BUTTON_LEFT:
                return NavigationEvent(subtype="pause")
            if event.button == cabot.button.BUTTON_RIGHT:
                return NavigationEvent(subtype="resume")
            if event.button == cabot.button.BUTTON_CENTER:
                return NavigationEvent(subtype="decision")
        if event.type == HoldDownEvent.TYPE:
            if event.holddown == cabot.button.BUTTON_LEFT:
                return NavigationEvent(subtype="idle")
        '''
        if event.button == cabot.button.BUTTON_SELECT:
                return NavigationEvent(subtype="pause")
        if event.type == "click":
            if event.buttons == cabot.button.BUTTON_SELECT and event.count == 2:
                return NavigationEvent(subtype="cancel")
            if event.buttons == cabot.button.BUTTON_NEXT and event.count == 2:
                return NavigationEvent(subtype="resume")
        '''
        return None


if __name__ == "__main__":
    rospy.init_node("cabot_ui_manager", log_level=rospy.DEBUG)
    try:
        CabotUIManager()
    except:
        rospy.logerr(traceback.format_exc())
    rospy.spin()
