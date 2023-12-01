#ifndef EVENT_HPP_
#define EVENT_HPP_

#include <map>
#include <string>
#include <stdexcept>
#include <vector>
#include <boost/algorithm/string.hpp>

class BaseEvent{
public:
  BaseEvent(std::string type = "");
  std::string getType() const;
  virtual std::string toString() const;
  static BaseEvent* parse(const std::string& text);
  static std::vector<std::unique_ptr<BaseEvent>>& getSubclasses();
private:
  virtual BaseEvent* _parse(const std::string& text = "", const std::string& type = "") = 0;
  std::string _type;
};

class ButtonEvent : public BaseEvent{
public:
  static const std::string TYPE;
  ButtonEvent(int button = 0, bool up = false, bool hold = false);
  bool operator==(const ButtonEvent& other) const;
  int get_button() const;
  bool is_up() const;
  bool is_down() const;
  bool is_hold() const;
  std::string toString() const override;
  BaseEvent* _parse(const std::string& text = "", const std::string& type = "") override;
  static BaseEvent* parse(const std::string& text);
private:
  int _button;
  bool _up;
  bool _hold;
};

class JoyButtonEvent : public ButtonEvent{
public:
  static const std::string TYPE;
  JoyButtonEvent(int button = 0, bool up = false, bool hold = false);
};

class ClickEvent : public BaseEvent{
public:
  static const std::string TYPE;
  ClickEvent(int buttons = 0, int count = 0);
  bool operator==(const ClickEvent& other) const;
  int get_buttons() const;
  int get_count() const;
  std::string toString() const override;
  BaseEvent* _parse(const std::string& text = "", const std::string& type = "") override;
  static BaseEvent* parse(const std::string& text);
private:
  int _buttons;
  int _count;
};

class HoldDownEvent : public BaseEvent{
public:
  static const std::string TYPE;
  HoldDownEvent(int holddown = 0);
  bool operator==(const HoldDownEvent& other) const;
  int get_holddown() const;
  std::string toString() const override;
  BaseEvent* _parse(const std::string& text = "", const std::string& type = "") override;
  static BaseEvent* parse(const std::string& text);
private:
  int _holddown;
};

class JoyClickEvent : public ClickEvent{
public:
  static const std::string TYPE;
  JoyClickEvent(int buttons = 0, int count = 0);
};

#endif // EVENT_HPP_
