// Copyright (c) 2023  Carnegie Mellon University
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#ifndef STOP_REASONER_HPP_
#define STOP_REASONER_HPP_

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <string>
#include <tuple>
#include <unordered_map>
#include <vector>

#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <people_msgs/msg/people.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>

namespace cabot_ui
{

enum class StopReason
{
  UNKNOWN,
  NO_NAVIGATION,
  NOT_STOPPED,
  NO_ODOMETORY,
  NO_TOUCH,
  STOPPED_BUT_UNDER_THRESHOLD,
  NO_CMD_VEL,
  THERE_ARE_PEOPLE_ON_THE_PATH,
  WAITING_FOR_ELEVATOR,
  AVOIDING_OBSTACLE,
  AVOIDING_PEOPLE,
  NONE
};

class StopReasonUtil
{
public:
  static std::string toStr(StopReason reason)
  {
    switch (reason) {
      case StopReason::UNKNOWN: return "UNKNOWN";
      case StopReason::NO_NAVIGATION: return "NO_NAVIGATION";
      case StopReason::NOT_STOPPED: return "NOT_STOPPED";
      case StopReason::NO_ODOMETORY: return "NO_ODOMETORY";
      case StopReason::NO_TOUCH: return "NO_TOUCH";
      case StopReason::STOPPED_BUT_UNDER_THRESHOLD: return "STOPPED_BUT_UNDER_THRESHOLD";
      case StopReason::NO_CMD_VEL: return "NO_CMD_VEL";
      case StopReason::THERE_ARE_PEOPLE_ON_THE_PATH: return "THERE_ARE_PEOPLE_ON_THE_PATH";
      case StopReason::WAITING_FOR_ELEVATOR: return "WAITING_FOR_ELEVATOR";
      case StopReason::AVOIDING_OBSTACLE: return "AVOIDING_OBSTACLE";
      case StopReason::AVOIDING_PEOPLE: return "AVOIDING_PEOPLE";
      case StopReason::NONE: return "NONE";
      default:
        return "NONE";
    }
  }
};

namespace Constant
{
static constexpr double FILTER_DURATION_SHORT = 1.0;
static constexpr double FILTER_DURATION_LONG = 5.0;
static constexpr double STOP_DURATION_THRESHOLD = 3.0;
static constexpr int REPLAN_REASON_COUNT = 2;
static constexpr double STOP_LINEAR_VELOCITY_THRESHOLD = 0.2;
static constexpr double STOP_ANGULAR_VELOCITY_THRESHOLD = 0.2;
}

template<typename DataT>
class FilterBase
{
public:
  explicit FilterBase(double duration)
  : duration_(seconds(duration), nanoseconds(duration))
  {
  }

  void clear()
  {
    history_.clear();
  }

  void input(rclcpp::Time time, DataT data)
  {
    history_.push_back(std::make_tuple(time, data));
    while (history_.size() > 0 && (time - std::get<0>(history_[0])) > duration_) {
      history_.erase(history_.begin());
    }
  }

  virtual DataT latest(rclcpp::Time now) = 0;
  rclcpp::Duration duration_since_latest(rclcpp::Time now)
  {
    if (history_.size() == 0) {return rclcpp::Duration(0, 0);}
    return now - history_.back().first;
  }

protected:
  rclcpp::Duration duration_;
  std::vector<std::tuple<rclcpp::Time, DataT>> history_;

private:
  int64_t seconds(double duration)
  {
    return static_cast<int64_t>(duration);
  }
  int64_t nanoseconds(double duration)
  {
    int64_t seconds = static_cast<int64_t>(duration);
    return static_cast<int64_t>((duration - seconds) * 1e9);
  }
};

class EnumFilter : public FilterBase<StopReason>
{
public:
  explicit EnumFilter(double duration)
  : FilterBase(duration)
  {
  }

  StopReason latest(rclcpp::Time now) override
  {
    if (history_.size() == 0) {return StopReason::NONE;}
    auto [time, data] = history_.back();
    if (now - time > duration_) {return StopReason::NONE;}
    return data;
  }

  StopReason majority(rclcpp::Time now)
  {
    std::unordered_map<StopReason, int> count;
    for (const auto & [time, value] : history_) {
      if (now - time > duration_) {
        continue;
      }
      count[value]++;
    }

    auto maxvote = StopReason::NONE;
    int maxcount = 0;
    for (const auto & [key, value] : count) {
      if (maxcount < value) {
        maxcount = value;
        maxvote = key;
      }
    }
    return maxvote;
  }
};

class AverageFilter : public FilterBase<double>
{
public:
  explicit AverageFilter(double duration)
  : FilterBase(duration)
  {
  }

  double latest(rclcpp::Time now) override
  {
    if (history_.size() == 0) {return -1.0;}
    auto [time, data] = history_.back();
    if (now - time > duration_) {return -1.0;}
    return data;
  }

  double average(rclcpp::Time now)
  {
    double average = 0;
    int count = 0;
    for (const auto & [time, value] : history_) {
      if (now - time > duration_) {
        continue;
      }
      average += value;
      count++;
    }
    if (count == 0) {
      return -1;
    }
    return average / count;
  }

  double minimum(rclcpp::Time now)
  {
    if (history_.size() == 0) {
      return -1;
    }
    auto [time, minimum] = history_.front();
    for (const auto & [time, value] : history_) {
      if (now - time > duration_) {
        continue;
      }
      if (value < minimum) {
        minimum = value;
      }
    }
    return minimum;
  }
};


class StopReasonFilter
{
public:
  static constexpr double EventInterval = 0.5;
  static constexpr double SummaryInterval = 15.0;

  explicit StopReasonFilter(std::vector<StopReason> ignore);
  void update(double duration, StopReason code);
  void conclude();
  std::tuple<double, StopReason> event();
  std::tuple<double, StopReason> summary();

private:
  StopReason prev_code_;
  double prev_event_duration_;
  double prev_summary_duration_;
  StopReason code_;
  double duration_;
  std::vector<StopReason> ignore_;
};

class StopReasoner
{
public:
  explicit StopReasoner(const std::shared_ptr<rclcpp::Node> node);
  void clear_history();
  void update_time(rclcpp::Time time);
  rclcpp::Time get_current_time();
  void input_odom(nav_msgs::msg::Odometry & msg);
  void input_event(std_msgs::msg::String & msg);
  void input_global_plan(nav_msgs::msg::Path & msg);
  void input_cmd_vel(geometry_msgs::msg::Twist & msg);
  void input_people_speed(std_msgs::msg::Float32 & msg);
  void input_touch_speed(std_msgs::msg::Float32 & msg);
  void input_replan_reason(people_msgs::msg::Person & msg);
  void input_current_frame(std_msgs::msg::String & msg);
  std::tuple<double, StopReason> update();
  void set_is_navigating(bool newValue);
  bool is_navigating();

private:
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener listener_;
  rclcpp::Logger logger_;
  rclcpp::Clock clock_;
  rclcpp::Time current_time_;
  bool is_sim_time_;
  bool is_stopped_;
  rclcpp::Time stopped_time_;
  StopReason prev_code_;
  bool is_navigating_;
  bool is_waiting_for_elevator_;
  rclcpp::Time navigation_timeout_;
  std::string last_log_;
  std::string current_frame_;
  AverageFilter linear_velocity_;
  AverageFilter angular_velocity_;
  AverageFilter cmd_vel_linear_;
  AverageFilter cmd_vel_angular_;
  AverageFilter people_speed_;
  AverageFilter touch_speed_;
  EnumFilter replan_reason_;
};
}  // namespace cabot_ui


#endif  // STOP_REASONER_HPP_
