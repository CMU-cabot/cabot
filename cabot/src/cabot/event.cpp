/*******************************************************************************
 * Copyright (c) 2023  Miraikan and Carnegie Mellon University
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *******************************************************************************/

#include "event.hpp"
#include <string>
#include <memory>
#include <vector>

BaseEvent::BaseEvent(std::string type)
: _type(type) {}
std::string BaseEvent::getType() const
{
  return _type;
}
std::string BaseEvent::toString() const
{
  throw std::runtime_error("event(" + _type + ")");
}
BaseEvent * BaseEvent::parse(const std::string & text)
{
  for (const std::unique_ptr<BaseEvent> & subclass : getSubclasses()) {
    BaseEvent * inst = subclass->_parse(text);
    if (inst != nullptr) {
      return inst;
    }
    inst = subclass->parse(text);
    if (inst != nullptr) {
      return inst;
    }
  }
  return nullptr;
}

BaseEvent * BaseEvent::_parse(const std::string&, const std::string&)
{
  return nullptr;
}

std::vector<std::unique_ptr<BaseEvent>> & BaseEvent::getSubclasses()
{
  static std::vector<std::unique_ptr<BaseEvent>> subclasses;
  return subclasses;
}

ButtonEvent::ButtonEvent(int button, bool up, bool hold)
: BaseEvent("button"), _button(button), _up(up), _hold(hold) {}
bool ButtonEvent::operator==(const ButtonEvent & other) const
{
  return getType() == other.getType() && _button == other._button && _up == other._up &&
         _hold == other._hold;
}
int ButtonEvent::get_button() const
{
  return _button;
}
bool ButtonEvent::is_up() const
{
  return !_hold && _up;
}
bool ButtonEvent::is_down() const
{
  return !_hold && !_up;
}
bool ButtonEvent::is_hold() const
{
  return _hold;
}
std::string ButtonEvent::toString() const
{
  std::string subtype = is_hold() ? "hold" : (is_up() ? "up" : "down");
  return getType() + "_" + subtype + "_" + std::to_string(_button);
}
BaseEvent * ButtonEvent::_parse(const std::string & text, const std::string&)
{
  if (text.find(TYPE) == 0) {
    std::vector<std::string> items;
    boost::split(items, text, boost::is_any_of("_"));
    bool hold = false;
    bool up = false;
    if (items[1] == "hold") {
      hold = true;
    }
    if (items[1] == "up") {
      up = true;
    }
    int button = std::stoi(items[2]);
    return new ButtonEvent(button, up, hold);
  }
  return nullptr;
}

const char* ButtonEvent::TYPE = "button";

JoyButtonEvent::JoyButtonEvent(int button, bool up, bool hold)
: ButtonEvent(button, up, hold) {}

const char* JoyButtonEvent::TYPE = "joybutton";

ClickEvent::ClickEvent(int buttons, int count)
: BaseEvent("click"), _buttons(buttons), _count(count) {}
bool ClickEvent::operator==(const ClickEvent & other) const
{
  return getType() == other.getType() && _buttons == other._buttons && _count == other._count;
}
int ClickEvent::get_buttons() const
{
  return _buttons;
}
int ClickEvent::get_count() const
{
  return _count;
}
std::string ClickEvent::toString() const
{
  return getType() + "_" + std::to_string(_buttons) + "_" + std::to_string(_count);
}
BaseEvent * ClickEvent::_parse(const std::string & text, const std::string&)
{
  if (text.find(TYPE) == 0) {
    std::vector<std::string> items;
    boost::split(items, text, boost::is_any_of("_"));
    int buttons = std::stoi(items[1]);
    int count = std::stoi(items[2]);
    return new ClickEvent(buttons, count);
  }
  return nullptr;
}

const char* ClickEvent::TYPE = "click";

HoldDownEvent::HoldDownEvent(int holddown)
: BaseEvent("holddown"), _holddown(holddown) {}
bool HoldDownEvent::operator==(const HoldDownEvent & other) const
{
  return getType() == other.getType() && _holddown == other._holddown;
}
int HoldDownEvent::get_holddown() const
{
  return _holddown;
}
std::string HoldDownEvent::toString() const
{
  return getType() + "_" + std::to_string(_holddown);
}

const char* HoldDownEvent::TYPE = "holddown";

BaseEvent * HoldDownEvent::_parse(const std::string & text, const std::string&)
{
  if (text.find(TYPE) == 0) {
    std::vector<std::string> items;
    boost::split(items, text, boost::is_any_of("_"));
    int holddown = std::stoi(items[1]);
    return new HoldDownEvent(holddown);
  }
  return nullptr;
}

JoyClickEvent::JoyClickEvent(int buttons, int count)
: ClickEvent(buttons, count) {}

const char* JoyClickEvent::TYPE = "joyclick";
