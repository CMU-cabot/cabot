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

#ifndef _PEOPLE_OBSTACLE_LAYER_H_
#define _PEOPLE_OBSTACLE_LAYER_H_

#include <rclcpp/rclcpp.hpp>
#include <nav2_costmap_2d/costmap_layer.hpp>
#include <nav2_costmap_2d/obstacle_layer.hpp>
#include <people_msgs/msg/people.hpp>
#include <people_msgs/msg/person.hpp>
#include <mutex>

namespace cabot_navigation2
{
  enum PeopleObstacleMode
  {
    NORMAL = 0,
    IGNORE_MOVE_PEOPLE,
    IGNORE_ALL_PEOPLE
  };

  class PeopleObstacleLayer : public nav2_costmap_2d::ObstacleLayer
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

    void updateCosts(nav2_costmap_2d::Costmap2D &master_grid,
                     int min_i, int min_j, int max_i, int max_j) override;

  private:
    rcl_interfaces::msg::SetParametersResult
    param_set_callback(const std::vector<rclcpp::Parameter> params);
    void peopleCallBack(const people_msgs::msg::People::SharedPtr people);
    void excludeCallBack(const people_msgs::msg::People::SharedPtr people);

    PeopleObstacleMode mode_;
    std::string people_topic_;
    std::string exclude_topic_;
    rclcpp::Subscription<people_msgs::msg::People>::SharedPtr people_sub_;
    rclcpp::Subscription<people_msgs::msg::People>::SharedPtr exclude_sub_;
    people_msgs::msg::People::SharedPtr last_people_;
    people_msgs::msg::People::SharedPtr last_exclude_;
    std::mutex mutex_;
    double person_radius_;
    bool people_enabled_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr callback_handler_;
  };
} // namespace cabot_navigation2
#endif
