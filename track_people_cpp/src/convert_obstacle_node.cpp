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

#include <ros/ros.h>
#include <obstacle_detector/Obstacles.h>
#include <track_people_py/TrackedBox.h>
#include <track_people_py/TrackedBoxes.h>

#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

namespace TrackObstacleCPP {
class ConvertObstacle {
 public:
  ConvertObstacle() : map_frame_name_("map") {}
  void onInit(ros::NodeHandle &nh);
  void message_callback(obstacle_detector::Obstacles::ConstPtr msg);
  std::string map_frame_name_;
  ros::Subscriber obstacle_sub_;
  ros::Publisher obstacle_pub_;
};

void ConvertObstacle::onInit(ros::NodeHandle &nh) {
  obstacle_sub_ = nh.subscribe("raw_obstacles", 10, &ConvertObstacle::message_callback, this);
  obstacle_pub_ = nh.advertise<track_people_py::TrackedBoxes>("track_people_py/detected_boxes", 10);

  if (nh.hasParam("map_frame")) {
    nh.getParam("map_frame", map_frame_name_);
  }
}

void ConvertObstacle::message_callback(obstacle_detector::Obstacles::ConstPtr msg) {
  track_people_py::TrackedBoxes boxes;

  boxes.camera_id = "obstacle_detector";
  boxes.header = msg->header;
  for (auto it = msg->circles.begin(); it != msg->circles.end(); it++) {
    track_people_py::TrackedBox box;
    box.center3d = it->center;
    box.box.Class = "obstacle";
    box.box.probability = 1.0;
    box.header = msg->header;
    boxes.tracked_boxes.push_back(box);
  }
  for (auto it = msg->segments.begin(); it != msg->segments.end(); it++) {
    track_people_py::TrackedBox box;
    auto dx = it->first_point.x - it->last_point.x;
    auto dy = it->first_point.y - it->last_point.y;
    auto dz = it->first_point.z - it->last_point.z;
    double max_size = 3.0;
    if (sqrt(dx*dx+dy*dy+dz*dz) < max_size) {
      box.center3d.x = (it->first_point.x + it->last_point.x)/2;
      box.center3d.y = (it->first_point.y + it->last_point.y)/2;
      box.center3d.z = (it->first_point.z + it->last_point.z)/2;
      box.box.Class = "obstacle";
      box.box.probability = 1.0;
      box.header = msg->header;
      boxes.tracked_boxes.push_back(box);
    }
  }

  obstacle_pub_.publish(boxes);
}
}  // namespace TrackObstacleCPP

int main(int argc, char **argv)
{
  ros::init(argc, argv, "convert_obstacle_node");
  ros::NodeHandle nh;

  TrackObstacleCPP::ConvertObstacle impl;
  impl.onInit(nh);

  ros::spin();
  return 0;
}
