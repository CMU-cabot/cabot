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

#include "detect_obstacle_on_path.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

namespace TrackObstacleCPP {
DetectObstacleOnPath::DetectObstacleOnPath() :
  map_frame_name_("map"),
  robot_frame_name_("base_footprint"),
  idx_(nullptr),
  data_(nullptr),
  footprint_size_(0.45),
  safety_margin_(0.25)
{}

void DetectObstacleOnPath::onInit(ros::NodeHandle &nh) {
  tfListener = new tf2_ros::TransformListener(tfBuffer);

  scan_sub_ = nh.subscribe("/scan", 10, &DetectObstacleOnPath::scanCallback, this);
  plan_sub_ = nh.subscribe("/plan", 10, &DetectObstacleOnPath::planCallback, this);
  obstacle_pub_ = nh.advertise<track_people_py::TrackedBoxes>("track_people_py/detected_boxes", 10);

  if (nh.hasParam("map_frame")) {
    nh.getParam("map_frame", map_frame_name_);
  }
  if (nh.hasParam("footprint_size")) {
    nh.getParam("footprint_size", footprint_size_);
  }
  if (nh.hasParam("safety_margin")) {
    nh.getParam("safety_margin", safety_margin_);
  }
}

void DetectObstacleOnPath::update() {
  if (last_plan_.header.frame_id.empty()) {
    ROS_INFO("last_plan_.header.frame_id.empty()");
    return;
  }

  ROS_INFO("update()");
  try {
    geometry_msgs::TransformStamped robotTransform =
        tfBuffer.lookupTransform(map_frame_name_, robot_frame_name_, ros::Time(0), ros::Duration(1.0));
    double robotx = robotTransform.transform.translation.x;
    double roboty = robotTransform.transform.translation.y;

    // find nearest pose on the path
    double total = 0;
    double min_dist = 1000;
    double min_index = last_pose_index_;
    geometry_msgs::PoseStamped &pose0 = last_plan_.poses[last_pose_index_];
    for (unsigned long i = last_pose_index_; i < last_plan_.poses.size(); i++) {
      geometry_msgs::PoseStamped &pose1 = last_plan_.poses[i];
      double px = pose1.pose.position.x;
      double py = pose1.pose.position.y;
      double d = std::hypot(robotx-px, roboty-py);
      if (d < min_dist) {
        min_dist = d;
        min_index = i;
      }
      double dist = std::hypot(pose0.pose.position.x - px, pose0.pose.position.y - py);
      total += dist;
      if (total > 2.0) {
        break;
      }
    }
    last_pose_index_ = min_index;

    track_people_py::TrackedBoxes boxes;
    boxes.camera_id = "scan";
    boxes.header.frame_id = map_frame_name_;
    boxes.header.stamp = last_scan_.header.stamp;

    float min_dist_l2 = std::pow(footprint_size_ + safety_margin_, 2);

    ROS_INFO("last_pose_index_=%ld robot=(%.2f,%.2f), plan_pose=(%.2f,%.2f)",
             last_pose_index_, robotx, roboty,
             last_plan_.poses[last_pose_index_].pose.position.x,
             last_plan_.poses[last_pose_index_].pose.position.y);
    for (unsigned long i = last_pose_index_; i < last_plan_.poses.size(); i++) {
      geometry_msgs::PoseStamped &pose = last_plan_.poses[i];
      cv::Mat query = cv::Mat::zeros(1, 2, CV_32FC1);
      float x = pose.pose.position.x;
      float y = pose.pose.position.y;
      query.at<float>(0) = x;
      query.at<float>(1) = y;
      std::vector<int> indices;
      std::vector<float> dists;

      //int m = idx_->radiusSearch(query, indices, dists, footprint_size_, 100);
      idx_->knnSearch(query, indices, dists, 10);
      if (dists.size() > 0) {
        for (unsigned long j = 0; j < dists.size() && j < indices.size(); j++) {
          float dist = dists[j];
          float sx = data_->at<float>(indices[j], 0);
          float sy = data_->at<float>(indices[j], 1);

          if (dist < min_dist_l2) {
            ROS_INFO("[%ld]:%.2f dist from (%.2f, %.2f) -> (%.2f, %.2f) = %.2f\n", j, dists[j], x, y, sx, sy, dist);
            track_people_py::TrackedBox box;
            box.header.frame_id = map_frame_name_;
            box.header.stamp = last_scan_.header.stamp;
            box.header.frame_id = map_frame_name_;
            box.header.stamp = last_scan_.header.stamp;
            box.box.Class = "obstacle";
            box.center3d.x = sx;
            box.center3d.y = sy;
            box.center3d.z = 0;
            boxes.tracked_boxes.push_back(box);
            break;
          }
        }
      }
      if (boxes.tracked_boxes.size() > 0) {
        break;
      }
    }
    obstacle_pub_.publish(boxes);
  } catch (std::exception e) {
    ROS_ERROR("Exception: %s", e.what());
  }
}

void DetectObstacleOnPath::scanCallback(sensor_msgs::LaserScan::ConstPtr msg) {
  last_scan_ = *msg;
  if (idx_) {
    delete idx_;
    idx_ = nullptr;
  }
  if (data_) {
    delete data_;
    data_ = nullptr;
  }
  int n = 0;
  for (unsigned long i = 0; i < msg->ranges.size(); i++) {
    float range = msg->ranges[i];
    if (range < msg->range_min || msg->range_max < range) continue;
    n++;
  }

  try {
    geometry_msgs::PoseStamped scanPointLocal;
    geometry_msgs::PoseStamped scanPointMap;
    scanPointLocal.header = msg->header;
    
    geometry_msgs::TransformStamped transformStamped = 
      tfBuffer.lookupTransform(map_frame_name_, msg->header.frame_id, msg->header.stamp, ros::Duration(1.0));

    data_ = new cv::Mat(n, 2, CV_32FC1);

    int j = 0;
    for (unsigned long i = 0; i < msg->ranges.size(); i++) {
      float range = msg->ranges[i];
      if (range < msg->range_min || msg->range_max < range) continue;
      float angle = msg->angle_min + msg->angle_increment * i;

      float x = std::cos(angle) * range;
      float y = std::sin(angle) * range;
      scanPointLocal.pose.position.x = x;
      scanPointLocal.pose.position.y = y;
      scanPointLocal.pose.position.z = 0;
      scanPointLocal.pose.orientation.x = 0;
      scanPointLocal.pose.orientation.y = 0;
      scanPointLocal.pose.orientation.z = 0;
      scanPointLocal.pose.orientation.w = 1;

      tf2::doTransform(scanPointLocal, scanPointMap, transformStamped);

      data_->at<float>(j, 0) = scanPointMap.pose.position.x;
      data_->at<float>(j, 1) = scanPointMap.pose.position.y;
      //ROS_INFO("%ld (%d) (%.2f, %.2f) (%.2f, %.2f) = (%.2f, %.2f)", i, j, 
      //range, angle, transformStamped.transform.translation.x, transformStamped.transform.translation.y, x, y);
      j++;
    }
    idx_ = new cv::flann::Index(*data_, cv::flann::KDTreeIndexParams(10), cvflann::FLANN_DIST_L2);

    update();
  } catch (tf2::ExtrapolationException e) {
    ROS_ERROR("extra ploration error %s", e.what());
  } catch (std::exception e) {
    ROS_ERROR("Exception: %s", e.what());
  }
}

void DetectObstacleOnPath::planCallback(nav_msgs::Path::ConstPtr msg) {
  ROS_INFO("planCallback");
  last_plan_ = nav_msgs::Path();
  try {
    geometry_msgs::TransformStamped transformStamped = tfBuffer.lookupTransform(
        map_frame_name_, msg->header.frame_id, msg->header.stamp, ros::Duration(1.0));
    last_plan_.header = msg->header;
    last_plan_.poses.clear();
    for (unsigned long i = 0; i < msg->poses.size(); i++) {
      geometry_msgs::PoseStamped pose;
      float x = transformStamped.transform.translation.x + msg->poses[i].pose.position.x;
      float y = transformStamped.transform.translation.y + msg->poses[i].pose.position.y;
      pose.header = msg->poses[i].header;
      pose.pose.position.x = x;
      pose.pose.position.y = y;
      pose.pose.position.z = 0;
      pose.pose.orientation = msg->poses[i].pose.orientation;
      last_plan_.poses.push_back(pose);
    }
    last_pose_index_ = 0;
  } catch (std::exception e) {
    ROS_ERROR("Exception: %s", e.what());
  }

  update();
}
}  // namespace TrackObstacleCPP
