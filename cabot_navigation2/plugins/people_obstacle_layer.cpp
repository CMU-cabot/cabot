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

#include <cabot_navigation2/people_obstacle_layer.h>
#include <pluginlib/class_list_macros.hpp>
#include <tf2/utils.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <chrono>
#include <cmath>
#include <algorithm>
#include <cctype>
#include <string>

PLUGINLIB_EXPORT_CLASS(cabot_navigation2::PeopleObstacleLayer, nav2_costmap_2d::Layer)

using nav2_costmap_2d::LETHAL_OBSTACLE;

namespace cabot_navigation2
{
  PeopleObstacleLayer::PeopleObstacleLayer() : mode_(PeopleObstacleMode::NORMAL),
                                               people_topic_("/people"),
                                               exclude_topic_("/exclude"),
                                               person_radius_(0.5),
                                               people_enabled_(true)
  {
  }

  PeopleObstacleLayer::~PeopleObstacleLayer()
  {
  }

  void PeopleObstacleLayer::activate()
  {
    nav2_costmap_2d::ObstacleLayer::activate();
  }

  void PeopleObstacleLayer::deactivate()
  {
    nav2_costmap_2d::ObstacleLayer::deactivate();
  }

  void PeopleObstacleLayer::reset()
  {
    nav2_costmap_2d::ObstacleLayer::reset();
  }

  void PeopleObstacleLayer::onInitialize()
  {
    auto node = node_.lock();

    RCLCPP_DEBUG(node->get_logger(), "ObstacleLayer::onInitialize");
    nav2_costmap_2d::ObstacleLayer::onInitialize();

    std::string mode = "normal";
    declareParameter("mode", rclcpp::ParameterValue(mode));
    node->get_parameter(name_ + "." + "mode", mode);
    std::transform(mode.begin(), mode.end(), mode.begin(),
                   [](unsigned char c) { return std::tolower(c); });

    if (mode == std::string("normal"))
    {
      mode_ = PeopleObstacleMode::NORMAL;
    }
    else if (mode == std::string("ignore"))
    {
      mode_ = PeopleObstacleMode::IGNORE;
    }
    else
    {
      mode_ = PeopleObstacleMode::NORMAL;
      RCLCPP_WARN(node->get_logger(), "Unknown mode string: '%s'. It should be normal or ignore ", mode.c_str());
    }
    declareParameter("people_topic", rclcpp::ParameterValue(people_topic_));
    node->get_parameter(name_ + "." + "people_topic", people_topic_);

    declareParameter("person_radius", rclcpp::ParameterValue(person_radius_));
    node->get_parameter(name_ + "." + "person_radius", person_radius_);

    declareParameter("people_enabled", rclcpp::ParameterValue(people_enabled_));
    node->get_parameter(name_ + "." + "people_enabled", people_enabled_);

    rclcpp::QoS people_qos(10);
    people_sub_ = node->create_subscription<people_msgs::msg::People>(
        people_topic_, people_qos,
        std::bind(&PeopleObstacleLayer::peopleCallBack, this, std::placeholders::_1));

    declareParameter("exclude_topic", rclcpp::ParameterValue(exclude_topic_));
    node->get_parameter(name_ + "." + "exclude_topic", exclude_topic_);

    exclude_sub_ = node->create_subscription<people_msgs::msg::People>(
        exclude_topic_, people_qos,
        std::bind(&PeopleObstacleLayer::excludeCallBack, this, std::placeholders::_1));

    callback_handler_ = node->add_on_set_parameters_callback(
        std::bind(&PeopleObstacleLayer::param_set_callback, this, std::placeholders::_1));

    RCLCPP_DEBUG(node->get_logger(), "PeopleObstacleLayer::onInitialize %d %d", size_x_, size_y_);
  }

  rcl_interfaces::msg::SetParametersResult
  PeopleObstacleLayer::param_set_callback(const std::vector<rclcpp::Parameter> params)
  {
    auto node = node_.lock();

    RCLCPP_INFO(node->get_logger(), "PeopleObstacleLayer::param_set_callback");
    auto results = std::make_shared<rcl_interfaces::msg::SetParametersResult>();

    for (auto &&param : params)
    {
      RCLCPP_INFO(node->get_logger(), "change param %s", param.get_name().c_str());
      if (!node->has_parameter(param.get_name()))
      {
        continue;
      }

      if (param.get_name() == name_ + ".people_enabled")
      {
        people_enabled_ = param.as_bool();
      }
    }
    results->successful = true;
    results->reason = "";
    return *results;
  }

  void PeopleObstacleLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double *min_x,
                                         double *min_y, double *max_x, double *max_y)
  {
    auto node = node_.lock();
    RCLCPP_DEBUG(node->get_logger(), "ObstacleLayer::updateBounds calling");
    nav2_costmap_2d::ObstacleLayer::updateBounds(robot_x, robot_y, robot_yaw, min_x, min_y, max_x, max_y);

    if (!people_enabled_)
    {
      RCLCPP_INFO(node->get_logger(), "ObstacleLayer people is disabled");
      return;
    }

    unsigned int x0, y0;
    if (!worldToMap(robot_x, robot_y, x0, y0))
    {
      RCLCPP_DEBUG(node->get_logger(), "robot is out of map");
      return;
    }
    if (!last_people_)
    {
      return;
    }

    geometry_msgs::msg::TransformStamped transform;
    try
    {
      RCLCPP_DEBUG(node->get_logger(), "lookupTransform %s->%s", last_people_->header.frame_id.c_str(), global_frame_.c_str());
      transform = tf_->lookupTransform(global_frame_, last_people_->header.frame_id, rclcpp::Time(0));
    }
    catch (tf2::TransformException & ex)
    {
      RCLCPP_ERROR(node->get_logger(), "%s", ex.what());
      return;
    }

    tf2::Transform tf2_transform;
    tf2::convert(transform.transform, tf2_transform);

    using namespace std::chrono;
    milliseconds s = duration_cast<milliseconds>(system_clock::now().time_since_epoch());
    std::lock_guard<std::mutex> lock(mutex_);

    double ASTEP = 0.02;
    double MAX_RANGE = 10.0 / layered_costmap_->getCostmap()->getResolution();

    MarkCell free_marker(costmap_, nav2_costmap_2d::FREE_SPACE);

    for (auto it = last_people_->people.begin(); it != last_people_->people.end(); it++)
    {
      bool exclude = false;
      if (last_exclude_)
      {
        for (auto it2 = last_exclude_->people.begin(); it2 != last_exclude_->people.end(); it2++)
        {
          if (it->name == it2->name)
          {
            exclude = true;
          }
        }
      }

      tf2::Vector3 p(it->position.x, it->position.y, 0);
      p = tf2_transform * p;

      //RCLCPP_DEBUG(node->get_logger(), "person velocity %.2f %.2f %.2f", it->velocity.x, it->velocity.y, it->velocity.z);
      // need to improve

      if ((mode_ == PeopleObstacleMode::IGNORE && !exclude) ||
          (mode_ == PeopleObstacleMode::NORMAL && exclude))
      {
        addExtraBounds(p.x() - person_radius_, p.y() - person_radius_, p.x() + person_radius_, p.y() + person_radius_);
        RCLCPP_INFO(node->get_logger(), "ignore person %s", it->name.c_str());
        for (double a = 0; a < M_PI * 2; a += ASTEP)
        {
          double tx = p.x() + person_radius_ * cos(a);
          double ty = p.y() + person_radius_ * sin(a);

          unsigned int x1, y1;
          if (!worldToMap(tx, ty, x1, y1))
          {
            RCLCPP_ERROR(node->get_logger(), "robot is out of map");
            continue;
          }

          raytraceLine(free_marker, x0, y0, x1, y1, MAX_RANGE);
        }
      }
      if ((mode_ == PeopleObstacleMode::NORMAL && !exclude) ||
          (mode_ == PeopleObstacleMode::IGNORE && exclude))
      {
        int scale = 1;
        addExtraBounds(p.x() - person_radius_ * scale, p.y() - person_radius_ * scale, p.x() + person_radius_ * scale, p.y() + person_radius_ * scale);
        RCLCPP_INFO(node->get_logger(), "include person %s", it->name.c_str());
        unsigned int x1, y1;
        if (worldToMap(p.x(), p.y(), x1, y1))
        {
          for (int i = 0; i >= 0; i--)
          {
            unsigned int cost = nav2_costmap_2d::LETHAL_OBSTACLE - 15 * i;
            MarkCell lethal_marker(costmap_, cost);

            double radius = person_radius_ * (8.0 + i) / 8.0;
            RCLCPP_INFO(node->get_logger(), "radius %.2f cost %u", radius, cost);
            for (double a = 0; a < M_PI * 2; a += ASTEP)
            {
              double tx = p.x() + radius * cos(a);
              double ty = p.y() + radius * sin(a);

              unsigned int x2, y2;
              if (!worldToMap(tx, ty, x2, y2))
              {
                RCLCPP_ERROR(node->get_logger(), "point is out of map, %.2f, %.2f, %.2f %.2f, %ud, %ud", p.x(), p.y(), tx, ty, x1, y1);
                continue;
              }
              raytraceLine(lethal_marker, x1, y1, x2, y2, MAX_RANGE);
            }
          }
        }
      }
    }

    milliseconds e = duration_cast<milliseconds>(system_clock::now().time_since_epoch());
    long long int duration = (e-s).count();
    RCLCPP_DEBUG(node->get_logger(), "PeopleObstacleLayer::updateBounds %lld ms", duration);

    useExtraBounds(min_x, min_y, max_x, max_y);
  }

  void PeopleObstacleLayer::updateCosts(nav2_costmap_2d::Costmap2D &master_grid,
                                        int min_i, int min_j, int max_i, int max_j)
  {
    auto node = node_.lock();
    nav2_costmap_2d::ObstacleLayer::updateCosts(master_grid, min_i, min_j, max_i, max_j);
    RCLCPP_DEBUG(node->get_logger(), "PeopleObstacleLayer::updateCosts, %d %d %d %d", min_i, min_j, max_i, max_j);
  }

  void PeopleObstacleLayer::peopleCallBack(const people_msgs::msg::People::SharedPtr people)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    last_people_ = people;
  }

  void PeopleObstacleLayer::excludeCallBack(const people_msgs::msg::People::SharedPtr exclude)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    last_exclude_ = exclude;
  }

} // namespace cabot_navigation2
