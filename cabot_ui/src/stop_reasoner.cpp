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

#include "stop_reasoner.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <vector>
#include <memory>

#include <geometry_msgs/msg/quaternion.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cabot/util.hpp>

namespace cabot_ui
{

StopReasonFilter::StopReasonFilter(std::vector<StopReason> ignore)
: prev_code_(StopReason::NONE),
  prev_event_duration_(0.0),
  prev_summary_duration_(0.0),
  code_(StopReason::NONE),
  duration_(0.0),
  ignore_(ignore)
{
}

void StopReasonFilter::update(double duration, StopReason code)
{
  duration_ = duration;
  code_ = code;
}

std::tuple<double, StopReason> StopReasonFilter::event()
{
  if (code_ == StopReason::NONE) {
    return std::make_tuple(0.0, StopReason::NONE);
  }
  if (prev_code_ != code_ ||
    duration_ - prev_event_duration_ > StopReasonFilter::EventInterval)
  {
    prev_event_duration_ = duration_;
    return std::make_tuple(duration_, code_);
  }
  return std::make_tuple(0.0, StopReason::NONE);
}

std::tuple<double, StopReason> StopReasonFilter::summary()
{
  if (std::find(ignore_.begin(), ignore_.end(), code_) == ignore_.end()) {
    if (prev_code_ != code_ ||
      duration_ - prev_summary_duration_ > StopReasonFilter::SummaryInterval)
    {
      prev_summary_duration_ = duration_;
      return std::make_tuple(duration_, code_);
    }
  }
  return std::make_tuple(0.0, StopReason::NONE);
}

void StopReasonFilter::conclude()
{
  prev_code_ = code_;
  duration_ = 0.0;
  code_ = StopReason::NONE;
}


StopReasoner::StopReasoner(const std::shared_ptr<rclcpp::Node> node)
: buffer_(node->get_clock(), tf2::durationFromSec(10)),
  listener_(buffer_),
  logger_(node->get_logger()),
  clock_(*node->get_clock()),
  current_time_(0, 0, RCL_SYSTEM_TIME),
  is_sim_time_(false),
  is_stopped_(false),
  stopped_time_(0, 0, RCL_SYSTEM_TIME),
  prev_code_(StopReason::NONE),
  is_navigating_(false),
  is_waiting_for_elevator_(false),
  navigation_timeout_(0),
  last_log_(""),
  current_frame_(""),
  linear_velocity_(Constant::FILTER_DURATION_SHORT),
  angular_velocity_(Constant::FILTER_DURATION_SHORT),
  cmd_vel_linear_(Constant::FILTER_DURATION_SHORT),
  cmd_vel_angular_(Constant::FILTER_DURATION_SHORT),
  people_speed_(Constant::FILTER_DURATION_LONG),
  touch_speed_(Constant::FILTER_DURATION_LONG),
  replan_reason_(Constant::FILTER_DURATION_LONG)
{
}

void StopReasoner::clear_history()
{
  people_speed_.clear();
  touch_speed_.clear();
  replan_reason_.clear();
}

void StopReasoner::update_time(rclcpp::Time time)
{
  current_time_ = time;
  is_sim_time_ = true;
}

rclcpp::Time StopReasoner::get_current_time()
{
  if (is_sim_time_) {
    return current_time_;
  }
  return clock_.now();
}

void StopReasoner::input_odom(nav_msgs::msg::Odometry & msg)
{
  auto lx = msg.twist.twist.linear.x;
  auto ly = msg.twist.twist.linear.y;
  auto az = msg.twist.twist.angular.z;
  linear_velocity_.input(get_current_time(), std::abs(std::sqrt(lx * lx + ly * ly)));
  angular_velocity_.input(get_current_time(), std::abs(az));
}

void StopReasoner::input_event(std_msgs::msg::String & msg)
{
  if (msg.data.substr(0, 6) == "button") {
    return;
  }
  if (msg.data.substr(0, 5) == "click") {
    return;
  }
  if (msg.data.substr(0, 10) == "navigation") {
    RCLCPP_DEBUG(logger_, "%.2f, %s", get_current_time().nanoseconds() / 1e9, msg.data.c_str());
  }

  if (msg.data == "navigation;event;navigation_start") {
    RCLCPP_DEBUG(logger_, "set_is_navigating(true)");
    set_is_navigating(true);
    is_waiting_for_elevator_ = false;
  } else if (msg.data == "navigation;event;waiting_for_elevator") {
    RCLCPP_DEBUG(logger_, "is_waiting_for_elevator_ = true;");
    is_waiting_for_elevator_ = true;
  } else if (msg.data == "navigation;event;elevator_door_may_be_ready") {
    RCLCPP_DEBUG(logger_, "is_waiting_for_elevator_ = false;");
    is_waiting_for_elevator_ = true;
  } else if (msg.data == "navigation_arrived") {
    RCLCPP_DEBUG(logger_, "set_is_navigating(false)");
    set_is_navigating(false);
  }
}

void StopReasoner::input_global_plan(nav_msgs::msg::Path & /* msg */)
{
  RCLCPP_INFO(logger_, "set_is_navigating(true)");
  set_is_navigating(true);
  navigation_timeout_ = get_current_time() + rclcpp::Duration(0.5, 0);
}

void StopReasoner::input_cmd_vel(geometry_msgs::msg::Twist & msg)
{
  cmd_vel_linear_.input(get_current_time(), std::abs(msg.linear.x));
  cmd_vel_angular_.input(get_current_time(), std::abs(msg.angular.z));
}

void StopReasoner::input_people_speed(std_msgs::msg::Float32 & msg)
{
  people_speed_.input(get_current_time(), msg.data);
}

void StopReasoner::input_touch_speed(std_msgs::msg::Float32 & msg)
{
  touch_speed_.input(get_current_time(), msg.data);
}

void StopReasoner::input_replan_reason(people_msgs::msg::Person & msg)
{
  try {
    auto transformStamped = buffer_.lookupTransform(
      current_frame_, "base_footprint", rclcpp::Time(0, 0, RCL_SYSTEM_TIME));
    auto translation = transformStamped.transform.translation;
    auto rotation = transformStamped.transform.rotation;

    tf2::Quaternion tf2_quaternion(rotation.x, rotation.y, rotation.z, rotation.w);
    tf2::Matrix3x3 rotation_matrix(tf2_quaternion);
    double roll, pitch, yaw;
    rotation_matrix.getRPY(roll, pitch, yaw);
    RCLCPP_DEBUG(logger_, "%.2f, %.2f, %.2f", translation.x, translation.y, translation.z);
    RCLCPP_DEBUG(logger_, "%.2f, %.2f, %.2f", roll, pitch, yaw);

    auto p0 = CaBotSafety::Point(translation.x, translation.y);
    auto p1 = CaBotSafety::Point(msg.position.x, msg.position.y);
    auto line = CaBotSafety::Line(p1, p0);
    auto q = line.quaternion();

    if (line.length() > 3.0) {
      RCLCPP_DEBUG(logger_, "too far");
      return;
    }

    if (std::abs(tf2_quaternion.angle(q)) > 90) {
      RCLCPP_DEBUG(logger_, "not in angle. %.2f", tf2_quaternion.angle(q));
      return;
    }

    if (msg.tags.size() == 0) {
      return;
    }

    RCLCPP_DEBUG(logger_, "%.2f, %.2f) %.2f", translation.x, translation.y, yaw);
    RCLCPP_DEBUG(logger_, "(%.2f, %.2f) %.2f", msg.position.x, msg.position.y, yaw);
    RCLCPP_DEBUG(logger_, "%.2f, %s", get_current_time().nanoseconds() / 1e9, msg.tagnames[0].c_str());
    if (msg.tagnames[0] == "avoiding obstacle") {
      replan_reason_.input(get_current_time(), StopReason::AVOIDING_OBSTACLE);
    }
    if (msg.tagnames[0] == "avoiding people") {
      replan_reason_.input(get_current_time(), StopReason::AVOIDING_PEOPLE);
    }
  } catch (std::exception & e) {
    RCLCPP_DEBUG(logger_, "%s", e.what());
  }
}

void StopReasoner::input_current_frame(std_msgs::msg::String & msg)
{
  current_frame_ = msg.data;
}

std::tuple<double, StopReason> StopReasoner::update()
{
  if (is_navigating_ == false) {
    is_stopped_ = false;
    stopped_time_ = rclcpp::Time(0, 0, RCL_SYSTEM_TIME);
    return std::make_tuple(0.0, StopReason::NO_NAVIGATION);
  }

  if (navigation_timeout_.nanoseconds() > 0 &&
    navigation_timeout_ < get_current_time())
  {
    is_navigating_ = false;
    navigation_timeout_ = rclcpp::Time(0, 0, RCL_SYSTEM_TIME);
  }

  // average velocity is under threshold
  if (linear_velocity_.latest(get_current_time()) < Constant::STOP_LINEAR_VELOCITY_THRESHOLD &&
    angular_velocity_.latest(get_current_time()) < Constant::STOP_ANGULAR_VELOCITY_THRESHOLD)
  {
    if (is_stopped_ == false) {
      is_stopped_ = true;
      stopped_time_ = get_current_time();
    }
  } else {
    is_stopped_ = false;
    stopped_time_ = rclcpp::Time(0, 0, RCL_SYSTEM_TIME);
  }

  double duration;
  if (stopped_time_.nanoseconds() > 0) {
    duration = (get_current_time() - stopped_time_).nanoseconds() / 1e9;
  } else {
    duration = -1.0;
  }

  // if (linear_velocity_.latest(get_current_time()) < 0 ||
  //     angular_velocity_.latest(get_current_time()) < 0) {
  //   return std::make_tuple(linear_velocity_.duration_since_latest(get_current_time()),
  //                          StopReason::NO_ODOMETORY);
  // }

  if (is_stopped_ == false) {
    return std::make_tuple(0.0, StopReason::NOT_STOPPED);
  }

  if (duration < Constant::STOP_DURATION_THRESHOLD) {
    return std::make_tuple(duration, StopReason::STOPPED_BUT_UNDER_THRESHOLD);
  }

  auto ts_latest = touch_speed_.latest(get_current_time());
  auto ts_average = touch_speed_.average(get_current_time());
  if (ts_latest >= 0 && ts_average >= 0 &&
    (ts_latest == 0 || ts_average < 1.0))
  {
    prev_code_ = StopReason::NO_TOUCH;
    return std::make_tuple(duration, StopReason::NO_TOUCH);
  }

  if (prev_code_ == StopReason::NO_TOUCH) {
    prev_code_ = StopReason::NONE;
    is_stopped_ = true;
    stopped_time_ = get_current_time();
    return update();
  }

  if (people_speed_.minimum(get_current_time()) >= 0) {
    if (people_speed_.minimum(get_current_time()) < 0.9) {
      RCLCPP_DEBUG(
        logger_, "%.2f, people_speed minimum=%.2f, average=%.2f",
        get_current_time().nanoseconds() / 1e9,
        people_speed_.minimum(get_current_time()),
        people_speed_.average(get_current_time()));
      return std::make_tuple(duration, StopReason::THERE_ARE_PEOPLE_ON_THE_PATH);
    }
  }

  if (is_waiting_for_elevator_) {
    return std::make_tuple(duration, StopReason::WAITING_FOR_ELEVATOR);
  }

  if (cmd_vel_linear_.latest(get_current_time()) == -1) {
    return std::make_tuple(duration, StopReason::NO_CMD_VEL);
  }

  if (replan_reason_.majority(get_current_time()) != StopReason::NONE) {
    return std::make_tuple(duration, replan_reason_.majority(get_current_time()));
  }

  return std::make_tuple(duration, StopReason::UNKNOWN);
}

void StopReasoner::set_is_navigating(bool newValue)
{
  if (newValue != is_navigating_) {
    RCLCPP_INFO(
      logger_, "%.2f, %s", get_current_time().nanoseconds() / 1e9,
      newValue ? "Navigation Started" : "Navigation Stopped");
    is_navigating_ = newValue;
    if (newValue == false) {
      clear_history();
    }
  }
}

bool StopReasoner::is_navigating()
{
  return is_navigating_;
}


}  // namespace cabot_ui
