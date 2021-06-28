// Copyright (c) 2020  Carnegie Mellon University
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

#include <action_bridge/action_bridge.hpp>

#ifdef __clang__
# pragma clang diagnostic push
# pragma clang diagnostic ignored "-Wunused-parameter"
#endif
#include <nav2_msgs/SpinAction.h>
#ifdef __clang__
# pragma clang diagnostic pop
#endif

// include ROS 2
#include <nav2_msgs/action/spin.hpp>

using Nav2SpinBridge = ActionBridge<nav2_msgs::SpinAction,
    nav2_msgs::action::Spin>;

template<>
void Nav2SpinBridge::translate_goal_1_to_2(const ROS1Goal & goal1, ROS2Goal & goal2)
{
  goal2.target_yaw = goal1.target_yaw;
}

template<>
void Nav2SpinBridge::translate_result_2_to_1(
  ROS1Result & result1,
  const ROS2Result & result2)
{
  result1.total_elapsed_time.data.sec = result2.total_elapsed_time.sec;
  result1.total_elapsed_time.data.nsec = result2.total_elapsed_time.nanosec;
}

template<>
void Nav2SpinBridge::translate_feedback_2_to_1(
  ROS1Feedback & feedback1,
  const ROS2Feedback & feedback2)
{
  feedback1.angular_distance_traveled = feedback2.angular_distance_traveled;
}

int main(int argc, char * argv[])
{
  return Nav2SpinBridge::main("spin", argc, argv);
}
