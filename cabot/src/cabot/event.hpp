#ifndef EVENT_HPP_
#define EVENT_HPP_

#include <string>
#include <stdexcept>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logging.hpp>
#include <boost/algorithm/string.hpp>
#include "cabot_handle_v2_node.hpp"

class CaBotHandleV2Node;

class BaseEvent{
public:
  BaseEvent(std::shared_ptr<CaBotHandleV2Node> node, std::string type = "");
  std::string getType() const;
  virtual std::string toString() const;
  /*static*/ std::shared_ptr<BaseEvent> parse(const std::string& text, std::shared_ptr<CaBotHandleV2Node> node);
  static std::vector<std::shared_ptr<BaseEvent>>& getSubclasses();
  std::shared_ptr<CaBotHandleV2Node> node_;
private:
  std::string _type;
  //std::shared_ptr<CaBotHandleV2Node> node_;
};

class ButtonEvent : public BaseEvent{
public:
  static const std::string TYPE;
  ButtonEvent(std::string type = "", std::shared_ptr<CaBotHandleV2Node> node = nullptr, int button = 0, bool up = false, bool hold = false);
  bool operator==(const ButtonEvent& other) const;
  int get_button() const;
  bool is_up() const;
  bool is_down() const;
  bool is_hold() const;
  std::string toString() const override;
  static std::shared_ptr<BaseEvent> _parse(const std::string& text, std::shared_ptr<CaBotHandleV2Node> node);
  std::shared_ptr<CaBotHandleV2Node> node_;
private:
  int _button;
  bool _up;
  bool _hold;
};

class JoyButtonEvent : public ButtonEvent{
public:
  static const std::string TYPE;
  JoyButtonEvent(std::shared_ptr<CaBotHandleV2Node> node, int button = 0, bool up = false, bool hold = false);
};

class ClickEvent : public BaseEvent{
public:
  static const std::string TYPE;
  ClickEvent(std::string type = "", std::shared_ptr<CaBotHandleV2Node> node = nullptr, int buttons = 0, int count = 0);
  bool operator==(const ClickEvent& other) const;
  int get_buttons() const;
  int get_count() const;
  std::string toString() const override;
  static std::shared_ptr<BaseEvent> _parse(const std::string& text, std::shared_ptr<CaBotHandleV2Node> node);
  std::shared_ptr<CaBotHandleV2Node> node_;
private:
  int _buttons;
  int _count;
};

class HoldDownEvent : public BaseEvent{
public:
  static const std::string TYPE;
  HoldDownEvent(std::string type = "", std::shared_ptr<CaBotHandleV2Node> node = nullptr, int holddown = 0);
  bool operator==(const HoldDownEvent& other) const;
  int get_holddown() const;
  std::string toString() const override;
  static std::shared_ptr<BaseEvent> _parse(const std::string& text, std::shared_ptr<CaBotHandleV2Node> node);
private:
  int _holddown;
};

class JoyClickEvent : public ClickEvent{
public:
  static const std::string TYPE;
  JoyClickEvent(std::shared_ptr<CaBotHandleV2Node> node, int buttons = 0, int count = 0);
};

#endif // EVENT_HPP_
