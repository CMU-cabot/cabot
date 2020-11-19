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

#include <cabot_navigation/people_obstacle_layer.h>
#include <pluginlib/class_list_macros.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <chrono>
#include <cmath>

PLUGINLIB_EXPORT_CLASS(cabot_navigation::PeopleObstacleLayer, costmap_2d::Layer)

using costmap_2d::LETHAL_OBSTACLE;

namespace cabot_navigation
{
  PeopleObstacleLayer::PeopleObstacleLayer() : people_topic_("/people"),
                                               vis_topic_("/visualize"),
                                               vis_frame_("map")
  {
  }

  PeopleObstacleLayer::~PeopleObstacleLayer()
  {
  }

  void PeopleObstacleLayer::activate()
  {
    ObstacleLayer::activate();
  }

  void PeopleObstacleLayer::deactivate()
  {
    ObstacleLayer::deactivate();
  }

  void PeopleObstacleLayer::reset()
  {
    ObstacleLayer::reset();
  }

  void PeopleObstacleLayer::onInitialize()
  {
    ObstacleLayer::onInitialize();

    ROS_DEBUG("PeopleObstacleLayer::onInitialize, %d", costmap_);
    ros::NodeHandle nh("~/" + name_), g_nh;

    nh.param("people_topic_", people_topic_, people_topic_);
    people_sub_ = nh.subscribe(people_topic_, 10, &PeopleObstacleLayer::peopleCallBack, this);

    nh.param("visualize_topic", vis_topic_, vis_topic_);
    vis_pub_ = nh.advertise<visualization_msgs::MarkerArray>(vis_topic_, 100);
    ROS_DEBUG("PeopleObstacleLayer::onInitialize %d %d", size_x_, size_y_);
  }

  void PeopleObstacleLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double *min_x,
                                         double *min_y, double *max_x, double *max_y)
  {

    ObstacleLayer::updateBounds(robot_x, robot_y, robot_yaw, min_x, min_y, max_x, max_y);
    auto now = ros::Time::now();
    ROS_DEBUG("PeopleObstacleLayer::updateBounds, (%.2f %.2f) %d.%d", robot_x, robot_y, now.sec, now.nsec);

    unsigned int x0, y0;
    if (!worldToMap(robot_x, robot_y, x0, y0))
    {
      ROS_ERROR("robot is out of map");
      return;
    }

    geometry_msgs::TransformStamped transform;
    try
    {
      transform = tf_->lookupTransform(last_people_.header.frame_id, global_frame_, ros::Time(0));
    }
    catch (tf2::TransformException ex)
    {
      ROS_ERROR("%s", ex.what());
      return;
    }

    tf2::Transform tf2_transform;
    tf2::convert(transform.transform, tf2_transform);

    using namespace std::chrono;
    milliseconds s = duration_cast<milliseconds>(system_clock::now().time_since_epoch());
    lock_.lock();

    double ASTEP = 0.02;
    double MAX_RANGE = 10.0 / layered_costmap_->getCostmap()->getResolution();
    double PERSON_SIZE = 0.5;
    MarkCell marker(costmap_, costmap_2d::FREE_SPACE);

    for (auto it = last_people_.people.begin(); it != last_people_.people.end(); it++)
    {
      tf2::Vector3 p(it->position.x, it->position.y, 0);
      p = tf2_transform * p;

      ROS_INFO("person velocity %.2f %.2f %.2f", it->velocity.x, it->velocity.y, it->velocity.z);
      // need to improve
      for (double a = 0; a < M_PI * 2; a += ASTEP)
      {
        double tx = p.x() + PERSON_SIZE * cos(a);
        double ty = p.y() + PERSON_SIZE * sin(a);

        unsigned int x1, y1;
        if (!worldToMap(tx, ty, x1, y1))
        {
          ROS_ERROR("robot is out of map");
          continue;
        }

        raytraceLine(marker, x0, y0, x1, y1, MAX_RANGE);
      }
    }

    lock_.unlock();
    milliseconds e = duration_cast<milliseconds>(system_clock::now().time_since_epoch());
    ROS_DEBUG("PeopleObstacleLayer::updateBounds %d ms", e - s);
  }

  void PeopleObstacleLayer::updateCosts(costmap_2d::Costmap2D &master_grid,
                                        int min_i, int min_j, int max_i, int max_j)
  {
    ObstacleLayer::updateCosts(master_grid, min_i, min_j, max_i, max_j);
    ROS_DEBUG("PeopleObstacleLayer::updateCosts, %d %d %d %d", min_i, min_j, max_i, max_j);
  }

  void PeopleObstacleLayer::peopleCallBack(const people_msgs::People::ConstPtr &people)
  {
    lock_.lock();
    last_people_ = *people;
    lock_.unlock();
  }

} // namespace cabot_navigation
