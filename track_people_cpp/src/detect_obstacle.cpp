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

#include "detect_obstacle.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

namespace TrackObstacleCPP {
DetectObstacle::DetectObstacle() : map_frame_name_("map"), idx_(nullptr), data_(nullptr), footprint_size_(0.45) {}

void DetectObstacle::onInit(ros::NodeHandle &nh) {
  tfListener = new tf2_ros::TransformListener(tfBuffer);

  scan_sub_ = nh.subscribe("/scan", 10, &DetectObstacle::scanCallback, this);
  plan_sub_ = nh.subscribe("/plan", 10, &DetectObstacle::planCallback, this);
  obstacle_pub_ = nh.advertise<track_people_py::TrackedBoxes>("track_people_py/detected_boxes", 10);

  if (nh.hasParam("map_frame")) {
    nh.getParam("map_frame", map_frame_name_);
  }
}

void DetectObstacle::update() {
  if (last_plan_.header.frame_id.empty()) {
    return;
  }

  try {
    geometry_msgs::TransformStamped transformStamped = tfBuffer.lookupTransform(
        map_frame_name_, last_plan_.header.frame_id, last_plan_.header.stamp, ros::Duration(1.0));
    track_people_py::TrackedBoxes boxes;
    boxes.camera_id = "scan";
    boxes.header.frame_id = map_frame_name_;
    boxes.header.stamp = last_scan_.header.stamp;
    for (unsigned long i = 0; i < last_plan_.poses.size(); i++) {
      geometry_msgs::PoseStamped &pose = last_plan_.poses[i];
      cv::Mat query = cv::Mat::zeros(1, 2, CV_32FC1);
      float x = transformStamped.transform.translation.x + pose.pose.position.x;
      float y = transformStamped.transform.translation.y + pose.pose.position.y;
      query.at<float>(0) = x;
      query.at<float>(1) = y;
      std::vector<int> indices;
      std::vector<float> dists;

      //int m = idx_->radiusSearch(query, indices, dists, footprint_size_, 100);
      idx_->knnSearch(query, indices, dists, 10);
      if (indices.size() > 0) {
        for (unsigned long j = 0; j < indices.size(); j++) {
          float sx = data_->at<float>(indices[j], 0);
          float sy = data_->at<float>(indices[j], 1);

          double dist = std::hypot(x - sx, y - sy);

          if (dist < footprint_size_) {
            ROS_INFO("[%ld]:%.2f dist from (%.2f, %.2f) -> (%.2f, %.2f) = %.2f\n", j, dists[j], x, y, sx, sy, dist);
            track_people_py::TrackedBox box;
            box.header.frame_id = map_frame_name_;
            box.header.stamp = last_scan_.header.stamp;
            box.header.frame_id = map_frame_name_;
            box.header.stamp = last_scan_.header.stamp;
            box.box.Class = "person";
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

void DetectObstacle::scanCallback(sensor_msgs::LaserScan::ConstPtr msg) {
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
  data_ = new cv::Mat(n, 2, CV_32FC1);

  try {
    geometry_msgs::PoseStamped scanPointLocal;
    geometry_msgs::PoseStamped scanPointMap;
    scanPointLocal.header = msg->header;
    
    geometry_msgs::TransformStamped transformStamped = 
      tfBuffer.lookupTransform(map_frame_name_, msg->header.frame_id, msg->header.stamp, ros::Duration(1.0));

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
    idx_ = new cv::flann::Index(*data_, cv::flann::KDTreeIndexParams(2), cvflann::FLANN_DIST_EUCLIDEAN);

    update();
  } catch (tf2::ExtrapolationException e) {
    ROS_ERROR("extra ploration error %s", e.what());
  }
}

void DetectObstacle::planCallback(nav_msgs::Path::ConstPtr msg) {
  last_plan_ = *msg;
  update();
}
}  // namespace TrackObstacleCPP
