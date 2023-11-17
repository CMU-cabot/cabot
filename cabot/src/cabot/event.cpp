#include "event.hpp"

BaseEvent::BaseEvent(std::string type)
  : _type(type){}
std::string BaseEvent::getType() const{
  return _type;
}
std::string BaseEvent::toString() const{
  throw std::runtime_error("event(" + _type + ")");
}
BaseEvent* BaseEvent::parse(const std::string&/* text */){
  return nullptr;
}
std::vector<BaseEvent*>& BaseEvent::getSubclasses(){
  static std::vector<BaseEvent*> subclasses;
  return subclasses;
}

ButtonEvent::ButtonEvent(std::string type, int button, bool up, bool hold)
  : BaseEvent(type.empty() ? TYPE : type), _button(button), _up(up), _hold(hold){}
bool ButtonEvent::operator==(const ButtonEvent& other) const{
  return getType() == other.getType() && _button == other._button && _up == other._up && _hold == other._hold;
}
int ButtonEvent::get_button() const{
  return _button;
}
bool ButtonEvent::is_up() const{
  return !_hold && _up;
}
bool ButtonEvent::is_down() const{
  return !_hold && !_up;
}
bool ButtonEvent::is_hold() const{
  return _hold;
}
std::string ButtonEvent::toString() const{
  std::string subtype = is_hold() ? "hold" : (is_up() ? "up" : "down");
  return getType() + "_" + subtype + "_" + std::to_string(_button);
}
BaseEvent* ButtonEvent::_parse(const std::string& text){
  if(text.find(TYPE) == 0){
    std::vector<std::string> items;
    boost::split(items, text, boost::is_any_of("_"));
    bool hold = false;
    bool up = false;
    if(items[1] == "hold"){
      hold = true;
    }
    if(items[1] == "up"){
      up = true;
    }
    int button = std::stoi(items[2]);
    return new ButtonEvent(TYPE, button, up, hold);
  }
  return nullptr;
}

const std::string ButtonEvent::TYPE = "button";

JoyButtonEvent::JoyButtonEvent(int button, bool up, bool hold)
  : ButtonEvent("joybutton", button, up, hold){}

const std::string JoyButtonEvent::TYPE = "joybutton";

ClickEvent::ClickEvent(std::string type, int buttons, int count)
  : BaseEvent(type.empty() ? TYPE : type), _buttons(buttons), _count(count){}
bool ClickEvent::operator==(const ClickEvent& other) const{
  return getType() == other.getType() && _buttons == other._buttons && _count == other._count;
}
int ClickEvent::get_buttons() const{
  return _buttons;
}
int ClickEvent::get_count() const{
  return _count;
}
std::string ClickEvent::toString() const{
  return getType() + "_" + std::to_string(_buttons) + "_" + std::to_string(_count);
}
BaseEvent* ClickEvent::_parse(const std::string& text){
  if(text.find(TYPE) == 0){
    std::vector<std::string> items;
    boost::split(items, text, boost::is_any_of("_"));
    int buttons = std::stoi(items[1]);
    int count = std::stoi(items[2]);
    return new ClickEvent(TYPE, buttons, count);
  }
  return nullptr;
}

const std::string ClickEvent::TYPE = "click";

HoldDownEvent::HoldDownEvent(std::string type, int holddown)
  : BaseEvent(type.empty() ? TYPE : type), _holddown(holddown){}
bool HoldDownEvent::operator==(const HoldDownEvent& other) const{
  return getType() == other.getType() && _holddown == other._holddown;
}
int HoldDownEvent::get_holddown() const{
  return _holddown;
}
std::string HoldDownEvent::toString() const{
  return getType() + "_" + std::to_string(_holddown);
}

const std::string HoldDownEvent::TYPE = "holddown";

BaseEvent* HoldDownEvent::parse(const std::string& text){
  if(text.find(TYPE) == 0){
    std::vector<std::string> items;
    boost::split(items, text, boost::is_any_of("_"));
    int holddown = std::stoi(items[1]);
    return new HoldDownEvent(TYPE, holddown);
  }
  return nullptr;
}

JoyClickEvent::JoyClickEvent(int buttons, int count)
  : ClickEvent("joyclick", buttons, count){}

const std::string JoyClickEvent::TYPE = "joyclick";
