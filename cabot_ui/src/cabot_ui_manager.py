#!/usr/bin/env python

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
"""
Cabot UI Manager

This class manages the state of the robot. 
It serves low level event and maps to high level event which may change state of the robot.

Ideally, this class has plugin architecture to add new UI component but it is not the current goal.
So, all controls which needs to see the current state of the robot are managed by this code.

Low-level (cabot.event) should be mapped into ui-level (cabot_ui.event)

Author: Daisuke Sato<daisuke@cmu.edu>
"""

import rospy
import std_msgs.msg

import cabot
import cabot.button
from cabot.event import BaseEvent, ButtonEvent, ClickEvent, JoyButtonEvent, JoyClickEvent
from cabot_ui.event import MenuEvent, NavigationEvent, ExplorationEvent
from cabot_ui.menu import Menu
from cabot_ui.status import State, StatusManager
from cabot_ui.interface import UserInterface
from cabot_ui.navigation import Navigation
from cabot_ui.exploration import Exploration


class CabotUIManager(object):
    def __init__(self):
        self.main_menu = Menu.create_menu({"menu":"main_menu"}, name_space=rospy.get_name())
        self.speed_menu = self.main_menu.get_menu_by_identifier("max_velocity_menu")

        init_speed = self.speed_menu.value
        try:
            init_speed = float(rospy.get_param("~init_speed", self.speed_menu.value))
        except ValueError:
            pass
 
        rospy.logdebug("Initial Speed = %.2f", init_speed)
        self.speed_menu.set_value(init_speed)
        
        rospy.loginfo(self.main_menu)
        self.main_menu.delegate = self
        self.menu_stack = []

        self.in_navigation = False
        self.destination = None

        rospy.Subscriber("/cabot/event", std_msgs.msg.String,
                         self._event_callback, None)
        #self.cancel_pub = rospy.Publisher("/navcog/cancel",
        #                         std_msgs.msg.Bool,
        #                         queue_size=10)
        
        #self.destination_pub = rospy.Publisher("/navcog/destination",
        #                                       std_msgs.msg.String,
        #                                       queue_size=10)

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

    ### navigation delegate
    def i_am_ready(self):
        self._interface.i_am_ready()

    def start_navigation(self):
        self._interface.start_navigation()

    def notify_turn(self, turn=None, pose=None):
        self._interface.notify_turn(turn=turn, pose=pose)

    def notify_human(self, angle=0, pose=None):
        self._interface.notify_human(angle=angle, pose=pose)

    def have_arrived(self):
        self._interface.have_arrived()

    def approaching_to_poi(self, poi=None, pose=None):
        self._interface.approaching_to_poi(poi=poi, pose=pose)

    def request_user_action(self, poi=None, pose=None):
        self._interface.request_user_action(poi=poi, pose=pose)

    def approached_to_poi(self, poi=None, pose=None):
        self._interface.approached_to_poi(poi=poi, pose=pose)

    def passed_poi(self, poi=None, pose=None):
        self._interface.passed_poi(poi=poi, pose=pose)

    ###
    def _event_callback(self, msg):
        event = BaseEvent.parse(msg.data)
        self.process_event(event)

    def reset(self):
        """reset menu"""
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
        rospy.logdebug("process_event %s", str(event))

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
            self.speed_menu.next()
            self._interface.menu_changed(menu=self.speed_menu)


        if event.subtype == "speeddown":
            self.speed_menu.prev()
            self._interface.menu_changed(menu=self.speed_menu)


        if event.subtype == "destination":
            #self.destination_pub.publish(event.param)
            self._navigation.set_destination(event.param)
            self.destination = event.param
            ## change state
            # change to waiting_action by using actionlib
            self._status_manager.set_state(State.in_action) 


        if event.subtype == "pause":
            self._interface.pause_navigation()
            #self.cancel_pub.publish(True)
            self._navigation.pause_navigation()


        if event.subtype == "cancel":
            self._interface.cancel_navigation()
            #self.cancel_pub.publish(True)
            self._navigation.cancel_navigation()
            self.in_navigation = False
            self.destination = None
            self._status_manager.set_state(State.idle)

        if event.subtype == "resume":
            self._interface.resume_navigation()            
            if self.destination is not None:
                #self.destination_pub.publish(self.destination)
                self._navigation.resume_navigation()

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

        if event.type != ButtonEvent.TYPE and event.type != ClickEvent.TYPE:
            return

        mevent = None
        if state == State.idle:
            mevent = self.map_button_to_menu(event)
            
        elif state == State.in_action or state == State.waiting_action:
            mevent = self.map_button_to_navigation(event)
            
        elif state == State.in_pause or state == State.waiting_pause:
            mevent = self.map_button_to_navigation(event)

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
            if event.button == cabot.button.BUTTON_NEXT or event.button == cabot.button.BUTTON_SPEED_UP:
                return NavigationEvent(subtype="speedup")
            if event.button == cabot.button.BUTTON_PREV or event.button == cabot.button.BUTTON_SPEED_DOWN:
                return NavigationEvent(subtype="speeddown")
            if event.button == cabot.button.BUTTON_SELECT:
                return NavigationEvent(subtype="pause")
        if event.type == "click":
            if event.buttons == cabot.button.BUTTON_SELECT and event.count == 2:
                return NavigationEvent(subtype="cancel")
            if event.buttons == cabot.button.BUTTON_NEXT and event.count == 2:
                return NavigationEvent(subtype="resume")
        return None


if __name__ == "__main__":
    rospy.init_node("cabot_ui_manager", log_level=rospy.DEBUG)
    CabotUIManager()
    rospy.spin()
