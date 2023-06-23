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

class BaseEvent(object):
    def __init__(self, type=None):
        self._type = type

    @property
    def type(self):
        return self._type

    def __str__(self):
        raise RuntimeError(F"event({self.type})")

    @classmethod
    def parse(cls, text):
        for c in cls.__subclasses__():
            # check the class has own _parse
            inst = c._parse(text)
            if inst is not None:
                return inst

            # otherwise check subclass parse
            inst = c.parse(text)
            if inst is not None:
                return inst

        return None


class ButtonEvent(BaseEvent):
    TYPE = "button"

    def __init__(self, type=None, button=0, up=False, hold=False):
        type = type if type is not None else ButtonEvent.TYPE
        super(ButtonEvent, self).__init__(type=type)
        self._button = button
        self._up = up
        self._hold = hold

    def __eq__(self, other):
        return self.type == other.type \
            and self.button == other.button \
            and self.up == other.up \
            and self.hold == other.hold

    @property
    def button(self):
        return self._button

    @property
    def up(self):
        if self._hold:
            return False
        return self._up

    @property
    def down(self):
        if self._hold:
            return False
        return not self._up

    @property
    def hold(self):
        return self._hold

    def __str__(self):
        subtype = "hold" if self.hold else "up" if self.up else "down"
        return F"{self.type}_{subtype}_{self.button}"

    @classmethod
    def _parse(cls, text):
        if text.startswith(cls.TYPE):
            items = text.split("_")
            hold = False
            up = False
            if items[1] == "hold":
                hold = True
            if items[1] == "up":
                up = True
            button = int(items[2])
            return cls(button=button, up=up, hold=hold)
        return None


class JoyButtonEvent(ButtonEvent):
    TYPE = "joybutton"

    def __init__(self, button=0, up=False, hold=False):
        super(JoyButtonEvent, self).__init__(type=JoyButtonEvent.TYPE,
                                             button=button, up=up, hold=hold)


class ClickEvent(BaseEvent):
    TYPE = "click"

    def __init__(self, type=None, buttons=0, count=0):
        type = type if type is not None else ClickEvent.TYPE
        super(ClickEvent, self).__init__(type=type)
        self._buttons = buttons
        self._count = count

    def __eq__(self, other):
        return self.type == other.type \
            and self.buttons == other.buttons \
            and self.count == other.count

    @property
    def buttons(self):
        return self._buttons

    @property
    def count(self):
        return self._count

    def __str__(self):
        return F"{self.type}_{self.buttons}_{self.count}"

    @classmethod
    def _parse(cls, text):
        if text.startswith(cls.TYPE):
            items = text.split("_")
            buttons = int(items[1])
            count = int(items[2])
            return cls(buttons=buttons, count=count)
        return None


class HoldDownEvent(BaseEvent):
    TYPE="holddown"
    def __init__(self, type=None, holddown=0):
        type = type if type is not None else HoldDownEvent.TYPE
        super(HoldDownEvent, self).__init__(type=type)
        self._holddown = holddown

    def __eq__(self, other):
        return self.type == other.type \
            and self.holddown == other.holddown

    @property
    def holddown(self):
        return self._holddown

    def __str__(self):
        return "%s_%d" % (self.type, self.holddown)

    @classmethod
    def _parse(cls, text):
        if text.startswith(cls.TYPE):
            items = text.split("_")
            holddown = int(items[1])
            return cls(holddown=holddown)
        return None


class JoyClickEvent(ClickEvent):
    TYPE = "joyclick"

    def __init__(self, buttons=0, count=0):
        super(JoyClickEvent, self).__init__(type=JoyClickEvent.TYPE,
                                            buttons=buttons, count=count)
