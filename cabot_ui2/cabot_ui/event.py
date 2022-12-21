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

import cabot.event


class SubtypeEvent(cabot.event.BaseEvent):
    TYPE = "subtype"

    def __init__(self, type=None, subtype=None, param=None):
        super(SubtypeEvent, self).__init__(type=type)
        self._subtype = subtype
        self._param = param

    def __eq__(self, other):
        return self.type == other.type \
            and self.subtype == other.subtype \
            and self.param == other.param

    @property
    def subtype(self):
        return self._subtype

    @property
    def param(self):
        return self._param

    def __str__(self):
        if self._param:
            return F"{self.type}_{self.subtype};{self.param}"
        return F"{self.type}_{self.subtype}"

    @classmethod
    def _parse(cls, text):
        if not text.startswith(cls.TYPE):
            return None

        items = text[len(cls.TYPE)+1:].split(";")
        subtype = items[0]
        param = None
        if len(items) == 2:
            param = items[1]

        return cls(subtype=subtype, param=param)


class MenuEvent(SubtypeEvent):
    TYPE = "menu"

    def __init__(self, subtype=None, param=None):
        super(MenuEvent, self).__init__(
            type=MenuEvent.TYPE, subtype=subtype, param=param)


class NavigationEvent(SubtypeEvent):
    TYPE = "navigation"

    def __init__(self, subtype=None, param=None):
        super(NavigationEvent, self).__init__(
            type=NavigationEvent.TYPE, subtype=subtype, param=param)


class ExplorationEvent(SubtypeEvent):
    TYPE = "exploration"

    def __init__(self, subtype=None, param=None):
        super(ExplorationEvent, self).__init__(
            type=ExplorationEvent.TYPE, subtype=subtype, param=param)
