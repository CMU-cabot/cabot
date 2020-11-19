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

#ifndef _PEOPLE_OBSTACLE_LAYER_H_
#define _PEOPLE_OBSTACLE_LAYER_H_

#include <ros/ros.h>
#include <costmap_2d/costmap_layer.h>
#include <costmap_2d/obstacle_layer.h>
#include <people_msgs/People.h>
#include <people_msgs/Person.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <boost/thread.hpp>

namespace cabot_navigation
{

  class PeopleObstacleLayer : public costmap_2d::ObstacleLayer
  {
  public:
    PeopleObstacleLayer();

    ~PeopleObstacleLayer() override;

    void activate() override;

    void deactivate() override;

    void reset() override;

    void onInitialize() override;

    void updateBounds(double robot_x, double robot_y, double robot_yaw,
                      double *min_x, double *min_y,
                      double *max_x, double *max_y) override;

    void updateCosts(costmap_2d::Costmap2D &master_grid,
                     int min_i, int min_j, int max_i, int max_j) override;

  private:
    void peopleCallBack(const people_msgs::People::ConstPtr &people);
    std::string people_topic_;
    ros::Subscriber people_sub_;
    people_msgs::People last_people_;
    boost::recursive_mutex lock_;

    // debug
    std::string vis_topic_;
    std::string vis_frame_;
    ros::Publisher vis_pub_;
    visualization_msgs::MarkerArray array;
    int vis_index = 0;
    // visualization functions
    // this also could be moved to somewhere else
    void clear()
    {
      visualization_msgs::Marker marker;
      marker.action = visualization_msgs::Marker::DELETEALL;
      array.markers.push_back(marker);
      commit();
      vis_index = 0;
    }

    void init_marker(visualization_msgs::Marker &marker, const std::string &frame_id,
                     float r = 0, float g = 0, float b = 0, float a = 1)
    {
      marker.header.stamp = ros::Time::now();
      marker.header.frame_id = frame_id;
      marker.id = vis_index++;
      marker.action = visualization_msgs::Marker::MODIFY;
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;
      marker.color.r = r;
      marker.color.g = g;
      marker.color.b = b;
      marker.color.a = a;
    }

    void add_point(double x, double y, const std::string &frame_id,
                   float size = 0.05, float r = 0, float g = 0, float b = 0, float a = 1)
    {
      visualization_msgs::Marker marker;
      init_marker(marker, frame_id, r, g, b, a);
      marker.ns = "laser_speed_control_point";
      marker.type = visualization_msgs::Marker::SPHERE;
      marker.pose.position.x = x;
      marker.pose.position.y = y;
      marker.scale.x = size;
      marker.scale.y = size;
      marker.scale.z = size;
      array.markers.push_back(marker);
    }

    void add_person(const people_msgs::Person &person, const std::string &frame_id,
                    float size = 0.25, float r = 0, float g = 0, float b = 0, float a = 1)
    {
      visualization_msgs::Marker marker;
      init_marker(marker, frame_id, r, g, b, a);
      marker.ns = "people_obstacle_person";
      marker.type = visualization_msgs::Marker::SPHERE;
      marker.pose.position.x = person.position.x;
      marker.pose.position.y = person.position.y;
      marker.scale.x = size;
      marker.scale.y = size;
      marker.scale.z = size;
      array.markers.push_back(marker);

      visualization_msgs::Marker text;
      init_marker(text, frame_id, r, g, b, a);
      text.ns = "people_obstacle_person_text";
      text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
      text.pose.position.x = person.position.x;
      text.pose.position.y = person.position.y;
      text.pose.position.z = 0.5;
      text.scale.z = size;
      text.text = person.name;
      array.markers.push_back(text);
    }

    void commit()
    {
      vis_pub_.publish(array);
      array.markers.clear();
    }
  };
} // namespace cabot_navigation
#endif
