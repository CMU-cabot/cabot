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
This is a menu implementation for cabot handle

Author: Daisuke Sato <daisukes@cmu.edu>
"""
import json
import subprocess
import os
import traceback

from cabot_ui.cabot_rclpy_util import CaBotRclpyUtil
import std_msgs.msg
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters, GetParameters
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.qos import QoSProfile, DurabilityPolicy
import rclpy.parameter

import cabot.util
from cabot_ui import i18n

class Action(object):
    """Menu Action abstract class"""
    def __init__(self, config, menu):
        self._menu = menu
        self._config = config

    def do_action(self):
        """need to implement do_action in concreate class"""
        return False

class Actions(Action):
    """Lisf of Actions"""
    @staticmethod
    def create_actions(config, menu):
        """create menu action classes"""
        actions = Menu.get_menu_config(config, "actions")

        return Actions(actions, menu)

    def __init__(self, config, menu):
        super(Actions, self).__init__(config, menu)
        temp = []
        if config:
            for action in config:
                _type = Menu.get_menu_config(action, "type", error=True)
                
                if _type == "publish_topic":
                    temp.append(PublishTopicAction(action, menu))
                elif _type == "reconfigure":
                    temp.append(ReconfigureAction(action, menu))
                elif _type == "syscommand":
                    temp.append(SyscommandAction(action, menu))
                else:
                    raise RuntimeError("%s action is not defined" % (_type))

        temp.append(MenuSelectAction(None, menu))

        self.actions = temp

    def do_action(self):
        result = True
        for action in self.actions:
            result = result and action.do_action()
        return result

    def __str__(self):
        return str(self.actions)

def my_import(name):
    components = name.split('.')
    mod = __import__(components[0])
    for comp in components[1:]:
        mod = getattr(mod, comp)
    return mod

class PublishTopicAction(Action):
    """Menu Action for publishing topic"""
    def __init__(self, config, menu):
        super(PublishTopicAction, self).__init__(config, menu)
        self._topic = Menu.get_menu_config(config, "topic", error=True)
        msg_type_str = Menu.get_menu_config(config, "msg_type", default="std_msgs.msg.String")
        self._msg_type = my_import(msg_type_str)
        
        if self._topic is not None:
            ### needs to update with custom message typep
            latching_qos = QoSProfile(depth=1,
                                      durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self._pub = CaBotRclpyUtil.instance().node.create_publisher(self._msg_type, self._topic, qos_profile=latching_qos)

    def do_action(self):
        curr = self._menu.value
        if curr is not None:
            if isinstance(curr, Menu):
                curr = curr.value
            if curr is not None:
                msg = self._msg_type()
                msg.data = curr
                self._pub.publish(msg)
                return True
        return False


class ReconfigureAction(Action):
    """Menu Action for reconfiguration"""
    def __init__(self, config, menu):
        super(ReconfigureAction, self).__init__(config, menu)
        self._targets = Menu.get_menu_config(config, "targets", error=True)
        self._error_count = 0

    _clients = {}

    def do_action(self):
        for target in self._targets:
            target_name = target["name"]
            if target_name not in ReconfigureAction._clients:
                try:
                    CaBotRclpyUtil.info("Trying to connect set_paraeters client")

                    ReconfigureAction._clients[target_name] = CaBotRclpyUtil.instance().node.create_client(SetParameters, f"/{target_name}/set_parameters")
                    ReconfigureAction._clients[target_name].wait_for_service(timeout_sec=3.0)
                except:
                    CaBotRclpyUtil.info("Timed out connecting set_parameters client")

            #return True
            if target_name in ReconfigureAction._clients:
                client = ReconfigureAction._clients[target_name]
                config = target["config"]
                if client is not None:
                    req = SetParameters.Request()
                    for key in config:
                        val = config[key]
                        if isinstance(val, (float,int)):
                            value = val * self._menu.value
                        elif isinstance(val, str):
                            # TODO (security issue)
                            value = eval(val)

                        param = Parameter()
                        param.name = key
                        param.value = ParameterValue(value)
                        req.parameters.append(param)

                    result = client.call(req)
                    CaBotRclpyUtil.info(f"{result}")
                    return True
        self._error_count += 1
        if self._error_count > 10:
            raise RuntimeError("dynamic_reconfigure server is not responded")
        return False

class SyscommandAction(Action):
    """Menu Action for system command"""
    def __init__(self, config, menu):
        super(SyscommandAction, self).__init__(config, menu)
        self._command = Menu.get_menu_config(config, "command", error=True)

    def do_action(self):
        try:
            CaBotRclpyUtil.info("do_action for system command")
            command = self._command % (self._menu.value)
            CaBotRclpyUtil.info(command)
            process = subprocess.Popen(command, preexec_fn=os.setsid, shell=True)
            process.wait(timeout=1)
        except:
            CaBotRclpyUtil.error(traceback.format_exc())
            return False
        return True

class Event(object):
    def __init__(self, origin, value):
        self.origin = origin
        self.value = value

class MenuSelectAction(Action):
    """Menu Select Action"""
    def __init__(self, config, menu):
        super(MenuSelectAction, self).__init__(config, menu)

    def do_action(self):
        self._menu._menu_selected(self._menu)
        return True

class Menu(object):
    """Menu class"""
    Undefined = 0
    List = 1
    Action = 2
    Adjust = 3

    def _get_path(self, name):
        return ".".join(["persistent", name])

    def _get_saved_config(self, name, default=None):
        try:
            self.get_client.wait_for_service(timeout_sec=3.0)
            path = self._get_path(name)
            
            req = GetParameters.Request()
            req.names.append(path)
            
            CaBotRclpyUtil.info(f"getting the param {path}")
            result = self.get_client.call(req)
            CaBotRclpyUtil.info(f"got the result {result}")
            if len(result.values) == 1:
                return rclpy.parameter.parameter_value_to_python(result.values[0])
            CaBotRclpyUtil.error("cannot get the parameter")
        except:
            if default is not None:
                self._save_config(name, default)
            return default

    def _save_config(self, name, value):
        CaBotRclpyUtil.info("save config")
        try:
            CaBotRclpyUtil.info(f"setting the param {name}: {value}")
            self.set_client.wait_for_service(timeout_sec=3.0)
            path = self._get_path(name)
            CaBotRclpyUtil.info(f"setting the param {path}: {value}")
            req = SetParameters.Request()
            param = rclpy.parameter.Parameter(path, value=value).to_parameter_msg()
            req.parameters.append(param)
            result = self.set_client.call(req)
            CaBotRclpyUtil.info(f"{result}")
        except:
            CaBotRclpyUtil.error(traceback.format_exc())
            CaBotRclpyUtil.error("cannot save the parameter")

    @staticmethod
    def get_menu_config(config, name, default=None, error=False):
        """Utility function to get config value specified by name.
        if value is not exists return 'default' value
        if error is True and value is not exists raise KeyError
        """
        if name in config:
            return config[name]
        elif error:
            raise KeyError("Config does not have '%s'"%name)
        return default

    @staticmethod
    def create_menu(menu_obj, config, identifier=None, title=None, usage=None, parent=None):
        if not config:
            return None
        
        """Create menu from config"""
        # refer menu
        menu = config["menu"] if "menu" in config else None
        if menu is not None:
            config2 = menu_obj[menu] if menu in menu_obj else []
            return Menu.create_menu(menu_obj, config2, identifier=menu, title=title, usage=usage, parent=parent)

        # otherwise
        _type = Menu.get_menu_config(config, "type", "item")

        if _type == "list":
            return MenuList(menu_obj, config, identifier=identifier, parent=parent)
        elif _type == "adjust":
            return MenuAdjust(config, identifier=identifier, parent=parent)
        elif _type == "item":
            return MenuItem(config, identifier=identifier, parent=parent)

        raise ValueError("%s is not a menu type" % (_type))


    def __init__(self, config=None, identifier=None, parent=None):
        self._title = Menu.get_menu_config(config, "title")
        self._usage = Menu.get_menu_config(config, "usage")
        self._type = Menu.Undefined
        self._config = config
        self._identifier = identifier
        self._parent = parent
        self._items = []
        self._actions = None
        self._listeners = []
        self.delegate = None
        CaBotRclpyUtil.info("creating SetParameters client")
        self.set_client = CaBotRclpyUtil.instance().node.create_client(SetParameters, "/parameter_server/set_parameters", callback_group=MutuallyExclusiveCallbackGroup())
        CaBotRclpyUtil.info("creating GetParameters client")
        self.get_client = CaBotRclpyUtil.instance().node.create_client(GetParameters, "/parameter_server/get_parameters", callback_group=MutuallyExclusiveCallbackGroup())
        CaBotRclpyUtil.info(f"menu initialization complete {self._title}")


    def __str__(self):
        text = ""
        if self._type == Menu.List:
            text += "Menu List (%s, %s)\n" % (self._identifier, self._title) \
                + "\n".join(["  "+str(x) for x in self._items])
        elif self._type == Menu.Action:
            text += "Menu Action (%s, %s)" % (self._identifier, self._title)
        elif self._type == Menu.Adjust:
            text += "Menu Adjust (%s, %s)" % (self._identifier, self._title)
        else:
            text += super(Menu, self).__str__()
        if self._actions is not None:
            text += "\n  with Action (%s)" % (self._actions)
        return text

    @property
    def identifier(self):
        """Menu identifier"""
        return self._identifier

    @property
    def type(self):
        """Menu type"""
        return self._type

    @property
    def title(self):
        """Menu title"""
        return i18n.localized_string(self._title)

    @property
    def usage(self):
        """Menu usage which is read by TTS"""
        return i18n.localized_string(self._usage)

    @property
    def description(self):
        """Description of the menu"""
        return i18n.localized_string(self._title)

    @property
    def value(self):
        """Value of the menu"""
        return None

    def sev_value(self, value):
        raise RuntimeError("not implemented")

    @property
    def can_explore(self):
        return False

    def next(self):
        """Move to next item or value"""
        pass

    def prev(self):
        """Move to previous item or value"""
        pass

    def select(self):
        """Do action for selection"""
        return self

    def reset(self):
        """Reset for reuse"""
        pass

    def _menu_selected(self, origin):
        """menu selected"""
        if self.delegate:
            self.delegate.menu_selected(origin)
        if self._parent is not None:
            self._parent._menu_selected(origin)


class MenuList(Menu):
    """List of Menu items"""
    def __init__(self, menu_obj, config=None, identifier=None, parent=None):
        if Menu.get_menu_config(config, "usage") is None:
            config["usage"] = "MENU_NAVIGATE_USAGE"
        super(MenuList, self).__init__(config=config, identifier=identifier, parent=parent)

        self._type = Menu.List
        self._actions = Actions.create_actions(config, self)

        temp = []
        items = Menu.get_menu_config(config, "items")
        for item in items:
            menu_item = Menu.create_menu(menu_obj, item, parent=self)
            if menu_item:
                temp.append(menu_item)
            else:
                CaBotRclpyUtil.error("menu {} is not found".format(item))
        self._items = temp
        self._current = None

    def _get_item(self, diff, default):
        if self._current is None:
            self._current = default
        else:
            self._current = (self._current + diff) % len(self._items)

        if self._current is None:
            return None
        return self._items[self._current]

    @property
    def value(self):
        """Current value"""
        return self._get_item(0, None)

    @property
    def can_explore(self):
        return True
    
    def next(self):
        return self._get_item(+1, 0)

    def prev(self):
        return self._get_item(-1, -1)

    def select(self):
        if self._actions is not None:
            self._actions.do_action()
        
        return self.value

    def get_menu_by_identifier(self, identifier):
        for item in self._items:
            if item._identifier == identifier:
                return item
        return None
    
    @property
    def description(self):
        #return self.value.title if self.value is not None else "not selected"\
        return i18n.localized_string(self.value._title) if self.value is not None else None

    def reset(self):
        self._current = None
        for item in self._items:
            item.reset()

class MenuAdjust(Menu):
    """Adjustable menu"""
    def __init__(self, config=None, identifier=None, parent=None):
        super(MenuAdjust, self).__init__(config=config, identifier=identifier, parent=parent)
        self._type = Menu.Adjust
        self._max = Menu.get_menu_config(config, "max", error=True)
        self._min = Menu.get_menu_config(config, "min", error=True)
        self._values = Menu.get_menu_config(config, "values")
        if self._values is not None:
            self._format = Menu.get_menu_config(config, "format", default="{}")
        else:
            self._format = Menu.get_menu_config(config, "format", default="{}")
        if self._min >= self._max:
            raise ValueError("min value should be smaller than max value " \
                             + "(%f < %f)"%(self._min, self._max))

        self._default = Menu.get_menu_config(config, "default", error=True)
        if self._default < self._min or self._max < self._default:
            raise ValueError("default value should be in min-max range " \
                             + "(%f < %f < %f" % (self._min,
                                                  self._default,
                                                  self._max))

        self._step = Menu.get_menu_config(config, "step", 1)
        self._name = Menu.get_menu_config(config, "name", error=True)
        self._current = self._get_saved_current()
        self._actions = Actions.create_actions(config, self)
        self._check_action_once()
        
    def _check_action(self):
        if self._actions is None:
            return
        if self._actions.do_action():
            return
        CaBotRclpyUtil.info("retry do_action with %s", str(self))
        self._check_action_once()

    @cabot.util.setInterval(3, 1)
    def _check_action_once(self):
        self._check_action()


    def _get_saved_current(self):
        temp = self._get_saved_config(self._name, default=self._default)
        if hasattr(self, "_values") and self._values is not None and isinstance(temp, str):
            temp = self._values.index(temp)
        return temp if temp is not None else self._default

    def _save_current(self):
        if self._actions is not None:
            self._actions.do_action()
        return self._save_config(self._name, self.value)

    @property
    def value(self):
        """Current value"""
        if self._values:
            return self._values[self._current]
        return self._current

    def set_value(self, value):
        if self._values:
            self._current = self._values.index(value)
        else:
            self._current = value
        self._save_current()

    @property
    def can_explore(self):
        return True

    @property
    def min(self):
        """Minimum value"""
        return self._min

    @property
    def max(self):
        """Maximum value"""
        return self._max

    ## intentionally opposite
    def next(self): 
        self._current = max(self._current - self._step, self._min)
        self._save_current()
        return self._current

    def prev(self):
        self._current = min(self._current + self._step, self._max)
        self._save_current()
        return self._current

    def select(self):
        return self

    @property
    def description(self):
        CaBotRclpyUtil.info(f"{self._format}, {self._current}, {self.value}")
        return i18n.localized_string(self._format, i18n.localized_string(self.value)) 
                #" " + i18n.localized_string(self._title))

    def reset(self):
        self._current = self._get_saved_current()

class MenuItem(Menu):
    """Menu item with action"""
    def __init__(self, config=None, identifier=None, parent=None):
        super(MenuItem, self).__init__(config=config, identifier=identifier, parent=parent)
        self._type = Menu.Action
        self._value = Menu.get_menu_config(config, "value")
        self._format = Menu.get_menu_config(config, "format", "MENU_ITEM_SELECTED")

    @property
    def description(self):
        return i18n.localized_string(self._format, i18n.localized_string(self._title))

    @property
    def value(self):
        return self._value

    @property
    def can_explore(self):
        return False

    def reset(self):
        pass

    def select(self):
        pass
