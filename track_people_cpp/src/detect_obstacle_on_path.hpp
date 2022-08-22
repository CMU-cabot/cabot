// Copyright (c) 2022  Carnegie Mellon University
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
//
// Author: Daisuke Sato <daisukes@cmu.edu>

#ifndef DETECT_OBSTACLE_HPP
#define DETECT_OBSTACLE_HPP

#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <ros/node_handle.h>
#include <ros/subscriber.h>
#include <ros/publisher.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf/LinearMath/Transform.h>
#include <tf/transform_datatypes.h>
#include <opencv2/flann/flann.hpp>
#include <track_people_py/TrackedBoxes.h>

namespace TrackObstacleCPP {
class DetectObstacleOnPath {
 public:
  DetectObstacleOnPath();
  void onInit(ros::NodeHandle &nh);

 private:
  void update();
  void scanCallback(sensor_msgs::LaserScan::ConstPtr msg);
  void planCallback(nav_msgs::Path::ConstPtr msg);

  std::string map_frame_name_;
  std::string robot_frame_name_;
  tf2_ros::TransformListener *tfListener;
  tf2_ros::Buffer tfBuffer;

  ros::Subscriber scan_sub_;
  ros::Subscriber plan_sub_;
  ros::Publisher obstacle_pub_;

  sensor_msgs::LaserScan last_scan_;
  nav_msgs::Path last_plan_;
  unsigned long last_pose_index_;

  cv::Mat *data_;
  cv::flann::Index *idx_;
  
  float footprint_size_;
  float safety_margin_;
};

}  // namespace TrackObstacleCPP

#endif
