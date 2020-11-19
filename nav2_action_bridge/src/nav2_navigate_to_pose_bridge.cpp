// Copyright (c) 2020 Carnegie Mellon University
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
//#include <move_base_msgs/MoveBaseAction.h>
#include <nav2_msgs/NavigateToPoseAction.h>
#ifdef __clang__
# pragma clang diagnostic pop
#endif

// include ROS 2
#include <nav2_msgs/action/navigate_to_pose.hpp>

using Nav2NavigateToPoseBridge = ActionBridge<nav2_msgs::NavigateToPoseAction,
    nav2_msgs::action::NavigateToPose>;

template<>
void Nav2NavigateToPoseBridge::translate_goal_1_to_2(const ROS1Goal & goal1, ROS2Goal & goal2)
{
  goal2.pose.header.stamp.sec = goal1.pose.header.stamp.sec;
  goal2.pose.header.stamp.nanosec = goal1.pose.header.stamp.nsec;
  goal2.pose.header.frame_id = goal1.pose.header.frame_id;
  goal2.pose.pose.position.x = goal1.pose.pose.position.x;
  goal2.pose.pose.position.y = goal1.pose.pose.position.y;
  goal2.pose.pose.position.z = goal1.pose.pose.position.z;
  goal2.pose.pose.orientation.x = goal1.pose.pose.orientation.x;
  goal2.pose.pose.orientation.y = goal1.pose.pose.orientation.y;
  goal2.pose.pose.orientation.z = goal1.pose.pose.orientation.z;
  goal2.pose.pose.orientation.w = goal1.pose.pose.orientation.w;
  goal2.behavior_tree = goal1.behavior_tree;
}

template<>
void Nav2NavigateToPoseBridge::translate_result_2_to_1(
  ROS1Result & result1,
  const ROS2Result & result2)
{
  //empty
}

template<>
void Nav2NavigateToPoseBridge::translate_feedback_2_to_1(
  ROS1Feedback & feedback1,
  const ROS2Feedback & feedback2)
{
  feedback1.current_pose.header.stamp.sec = feedback2.current_pose.header.stamp.sec;
  feedback1.current_pose.header.stamp.nsec = feedback2.current_pose.header.stamp.nanosec;
  feedback1.current_pose.header.frame_id = feedback2.current_pose.header.frame_id;
  feedback1.current_pose.pose.position.x = feedback2.current_pose.pose.position.x;
  feedback1.current_pose.pose.position.y = feedback2.current_pose.pose.position.y;
  feedback1.current_pose.pose.position.z = feedback2.current_pose.pose.position.z;
  feedback1.current_pose.pose.orientation.x = feedback2.current_pose.pose.orientation.x;
  feedback1.current_pose.pose.orientation.y = feedback2.current_pose.pose.orientation.y;
  feedback1.current_pose.pose.orientation.z = feedback2.current_pose.pose.orientation.z;
  feedback1.current_pose.pose.orientation.w = feedback2.current_pose.pose.orientation.w;
  feedback1.navigation_time.data.sec = feedback2.navigation_time.sec;
  feedback1.navigation_time.data.nsec = feedback2.navigation_time.nanosec;
  feedback1.number_of_recoveries = feedback2.number_of_recoveries;
  feedback1.distance_remaining = feedback2.distance_remaining;
}

int main(int argc, char * argv[])
{
  std::string service = "navigate_to_pose";

  if (argc > 1) {
    service = std::string(argv[1]);
  }
  
  return Nav2NavigateToPoseBridge::main(service, argc, argv);
}
