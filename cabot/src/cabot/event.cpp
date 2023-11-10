#include "event.hpp"

BaseEvent::BaseEvent(std::shared_ptr<CaBotHandleV2Node> node, std::string type)
  : _type(type), node_(node){}

std::string BaseEvent::getType() const{
  return _type;
}
std::string BaseEvent::toString() const{
  throw std::runtime_error("event(" + _type + ")");
}
std::shared_ptr<BaseEvent> BaseEvent::parse(const std::string& text, std::shared_ptr<CaBotHandleV2Node> node){
  node_ = node;
  for(const std::shared_ptr<BaseEvent>& subclass : getSubclasses()){
    std::shared_ptr<BaseEvent> inst = subclass->parse(text, node);
    if(inst){
      return inst;
    }
  }
  return nullptr;
}
std::vector<std::shared_ptr<BaseEvent>>& BaseEvent::getSubclasses(){
  static std::vector<std::shared_ptr<BaseEvent>> subclasses;
  return subclasses;
}

ButtonEvent::ButtonEvent(std::string type, std::shared_ptr<CaBotHandleV2Node> node, int button, bool up, bool hold)
  : BaseEvent(node, type.empty() ? TYPE : type), node_(node), _button(button), _up(up), _hold(hold){}
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
std::shared_ptr<BaseEvent> ButtonEvent::_parse(const std::string& text, std::shared_ptr<CaBotHandleV2Node> node){
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
    std::shared_ptr<ButtonEvent> inst_parse = std::make_shared<ButtonEvent>(TYPE, node, button, up, hold);
    if(!inst_parse){
      RCLCPP_ERROR(node->get_logger(), "Failed to allocate memory .");
      return nullptr;
    }
    return inst_parse;
  }
  return nullptr;
}

const std::string ButtonEvent::TYPE = "button";

JoyButtonEvent::JoyButtonEvent(std::shared_ptr<CaBotHandleV2Node> node, int button, bool up, bool hold)
  : ButtonEvent(TYPE, node, button, up, hold){}

const std::string JoyButtonEvent::TYPE = "joybutton";

ClickEvent::ClickEvent(std::string type, std::shared_ptr<CaBotHandleV2Node> node,  int buttons, int count)
  : BaseEvent(node, type.empty() ? TYPE : type), node_(node), _buttons(buttons), _count(count){}
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
std::shared_ptr<BaseEvent> ClickEvent::_parse(const std::string& text, std::shared_ptr<CaBotHandleV2Node> node){
  if(text.find(TYPE) == 0){
    std::vector<std::string> items;
    boost::split(items, text, boost::is_any_of("_"));
    int buttons = std::stoi(items[1]);
    int count = std::stoi(items[2]);
    std::shared_ptr<ClickEvent> inst_parse = std::make_shared<ClickEvent>(TYPE, node, buttons, count);
    if(!inst_parse){
      RCLCPP_ERROR(node->get_logger(), "Failed to allocate memory .");
      return nullptr;
    }
    return inst_parse;
  }
  return nullptr;
}

const std::string ClickEvent::TYPE = "click";

HoldDownEvent::HoldDownEvent(std::string type, std::shared_ptr<CaBotHandleV2Node> node, int holddown)
  : BaseEvent(node, type.empty() ? TYPE : type), _holddown(holddown){}
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

std::shared_ptr<BaseEvent> HoldDownEvent::_parse(const std::string& text, std::shared_ptr<CaBotHandleV2Node> node){
  if(text.find(TYPE) == 0){
    std::vector<std::string> items;
    boost::split(items, text, boost::is_any_of("_"));
    int holddown = std::stoi(items[1]);
    std::shared_ptr<HoldDownEvent> inst_parse = std::make_shared<HoldDownEvent>(TYPE, node, holddown);
    if(!inst_parse){
      RCLCPP_ERROR(node->get_logger(), "Failed to allocate memory .");
      return nullptr;
    }
    return inst_parse;
  }
  return nullptr;
}

JoyClickEvent::JoyClickEvent(std::shared_ptr<CaBotHandleV2Node> node, int buttons, int count)
  : ClickEvent(TYPE, node, buttons, count){}

const std::string JoyClickEvent::TYPE = "joyclick";
