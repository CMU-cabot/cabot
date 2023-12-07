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

#ifndef CABOT__EVENT_HPP_
#define CABOT__EVENT_HPP_

#include <boost/algorithm/string.hpp>
#include <map>
#include <string>
#include <stdexcept>
#include <vector>
#include <memory>

class BaseEvent
{
public:
  explicit BaseEvent(std::string type = "");
  std::string getType() const;
  virtual std::string toString() const;
  static BaseEvent * parse(const std::string & text);
  static std::vector<std::unique_ptr<BaseEvent>> & getSubclasses();

private:
  virtual BaseEvent * _parse(const std::string & text = "", const std::string & type = "") = 0;
  std::string _type;
};

class ButtonEvent : public BaseEvent
{
public:
  static const char* TYPE;
  explicit ButtonEvent(int button = 0, bool up = false, bool hold = false);
  bool operator==(const ButtonEvent & other) const;
  int get_button() const;
  bool is_up() const;
  bool is_down() const;
  bool is_hold() const;
  std::string toString() const override;
  BaseEvent * _parse(const std::string & text = "", const std::string & type = "") override;
  static BaseEvent * parse(const std::string & text);

private:
  int _button;
  bool _up;
  bool _hold;
};

class JoyButtonEvent : public ButtonEvent
{
public:
  static const char* TYPE;
  explicit JoyButtonEvent(int button = 0, bool up = false, bool hold = false);
};

class ClickEvent : public BaseEvent
{
public:
  static const char* TYPE;
  explicit ClickEvent(int buttons = 0, int count = 0);
  bool operator==(const ClickEvent & other) const;
  int get_buttons() const;
  int get_count() const;
  std::string toString() const override;
  BaseEvent * _parse(const std::string & text = "", const std::string & type = "") override;
  static BaseEvent * parse(const std::string & text);

private:
  int _buttons;
  int _count;
};

class HoldDownEvent : public BaseEvent
{
public:
  static const char* TYPE;
  explicit HoldDownEvent(int holddown = 0);
  bool operator==(const HoldDownEvent & other) const;
  int get_holddown() const;
  std::string toString() const override;
  BaseEvent * _parse(const std::string & text = "", const std::string & type = "") override;
  static BaseEvent * parse(const std::string & text);

private:
  int _holddown;
};

class JoyClickEvent : public ClickEvent
{
public:
  static const char* TYPE;
  explicit JoyClickEvent(int buttons = 0, int count = 0);
};

#endif  // CABOT__EVENT_HPP_
