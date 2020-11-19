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

// People speed control
// Author: Daisuke Sato <daisukes@cmu.edu>

#include <limits>
#include <sstream>
#include <cmath>

#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <ros/time.h>

#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/utils.h>

#include <people_msgs/People.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include "cabot/util.hpp"

namespace Safety
{
  // utility struct and functions
  // this could be moved to somewhere else
  class PeopleSpeedControlNodelet : public nodelet::Nodelet
  {

  public:
    std::string people_topic_;
    std::string vis_topic_;
    std::string limit_topic_;
    std::string odom_topic_;
    std::string plan_topic_;
    std::string event_topic_;
    std::string set_social_distance_topic_;
    std::string get_social_distance_topic_;

    std::string map_frame_;
    std::string robot_base_frame_;

    double max_speed_;
    double min_speed_;
    double max_acc_;
    double delay_;
    double social_distance_x_;
    double social_distance_y_;

    ros::Subscriber people_sub_;
    ros::Subscriber odom_sub_;
    ros::Subscriber plan_sub_;
    ros::Publisher vis_pub_;
    ros::Publisher limit_pub_;
    ros::Publisher event_pub_;
    ros::Subscriber set_social_distance_sub_;
    ros::Publisher get_social_distance_pub_;
    boost::mutex thread_sync_;
    tf2_ros::TransformListener *tfListener;
    tf2_ros::Buffer tfBuffer;
    nav_msgs::Odometry last_odom_;
    nav_msgs::Path last_plan_;

    PeopleSpeedControlNodelet()
        : people_topic_("/people"),
          vis_topic_("/visualize"),
          limit_topic_("/people_limit"),
          odom_topic_("/odom"),
          plan_topic_("/plan"),
          event_topic_("/event"),
          set_social_distance_topic_("/set_social_distance"),
          get_social_distance_topic_("/get_social_distance"),
          map_frame_("map"),
          robot_base_frame_("base_footprint"),
          max_speed_(1.0),
          min_speed_(0.1),
          max_acc_(0.5),
          delay_(0.5),
          social_distance_x_(2.0),
          social_distance_y_(1.0)
    {
      ROS_INFO("PeopleSpeedControlNodeletClass Constructor");
      tfListener = new tf2_ros::TransformListener(tfBuffer);
    }

    ~PeopleSpeedControlNodelet()
    {
      ROS_INFO("PeopleSpeedControlNodeletClass Destructor");
    }

  private:
    void onInit()
    {
      ROS_INFO("People speed control - %s", __FUNCTION__);
      ros::NodeHandle &private_nh = getPrivateNodeHandle();

      private_nh.getParam("people_topic", people_topic_);
      people_sub_ = private_nh.subscribe(people_topic_, 10, &PeopleSpeedControlNodelet::peopleCallback, this);

      private_nh.getParam("odom_topic", odom_topic_);
      odom_sub_ = private_nh.subscribe(odom_topic_, 10, &PeopleSpeedControlNodelet::odomCallback, this);

      private_nh.getParam("plan_topic", plan_topic_);
      plan_sub_ = private_nh.subscribe(plan_topic_, 10, &PeopleSpeedControlNodelet::planCallback, this);

      private_nh.getParam("visualize_topic", vis_topic_);
      vis_pub_ = private_nh.advertise<visualization_msgs::MarkerArray>(vis_topic_, 100);

      private_nh.getParam("limit_topic", limit_topic_);
      limit_pub_ = private_nh.advertise<std_msgs::Float32>(limit_topic_, 100);

      private_nh.getParam("event_topic", event_topic_);
      event_pub_ = private_nh.advertise<std_msgs::String>(event_topic_, 100);

      private_nh.getParam("max_speed_", max_speed_);
      private_nh.getParam("min_speed_", min_speed_);
      private_nh.getParam("max_acc_", max_acc_);
      private_nh.getParam("social_distance_x", social_distance_x_);
      private_nh.getParam("social_distance_y", social_distance_y_);

      ROS_INFO("PeopleSpeedControl with max_speed=%.2f, social_distance=(%.2f, %.2f)",
               max_speed_, social_distance_x_, social_distance_y_);

      if (private_nh.hasParam("set_social_distance_topic"))
        private_nh.getParam("set_social_distance_topic", set_social_distance_topic_);
      set_social_distance_sub_ = private_nh.subscribe(set_social_distance_topic_, 10, &PeopleSpeedControlNodelet::setSocialDistanceCallback, this);

      if (private_nh.hasParam("get_social_distance_topic"))
        private_nh.getParam("get_social_distance_topic", get_social_distance_topic_);
      get_social_distance_pub_ = private_nh.advertise<geometry_msgs::Point>(get_social_distance_topic_, 100, true);
      geometry_msgs::Point msg;
      msg.x = social_distance_x_;
      msg.y = social_distance_y_;
      get_social_distance_pub_.publish(msg);

      NODELET_INFO("PeopleSpeedControl with max_speed=%.2f, social_distance=(%.2f, %.2f)", max_speed_, social_distance_x_, social_distance_y_);
    }

    void peopleCallback(const people_msgs::People::ConstPtr &input)
    {
      if (last_plan_.poses.size() == 0)
      {
        NODELET_INFO("PeopleSpeedControl no plan");
        return;
      }

      geometry_msgs::TransformStamped transform_msg;
      try
      {
        transform_msg = tfBuffer.lookupTransform("base_footprint", "map",
                                                 ros::Time(0), ros::Duration(1.0));
      }
      catch (tf2::TransformException &ex)
      {
        NODELET_WARN("%s", ex.what());
        return;
      }
      tf2::Stamped<tf2::Transform> transform_tf2;
      tf2::fromMsg(transform_msg, transform_tf2);
      auto rotation_tf2 = transform_tf2;
      rotation_tf2.setOrigin(tf2::Vector3(0, 0, 0));

      double speed_limit = max_speed_;

      boost::mutex::scoped_lock lock(thread_sync_);
      for (auto it = input->people.begin(); it != input->people.end(); it++)
      {
        tf2::Vector3 p_frame(it->position.x, it->position.y, 0);
        tf2::Vector3 v_frame(it->velocity.x, it->velocity.y, 0);

        auto p_local = transform_tf2 * p_frame;
        auto v_local = rotation_tf2 * v_frame;

        double x = p_local.x();
        double y = p_local.y();
        double vx = v_local.x();
        double vy = v_local.y();
        double dist = sqrt(x * x + y * y);

        double pt = atan2(y, x);
        double sdx = abs(social_distance_x_ * cos(pt));
        double sdy = abs(social_distance_y_ * sin(pt));
        double min_path_dist = 100;

        if (abs(pt) > M_PI_2)
        {
          continue;
        }
        auto max_v = [](double D, double A, double d) {
          return (-2 * A * d + sqrt(4 * A * A * d * d + 8 * A * D)) / 2;
        };

        for (auto pose : last_plan_.poses)
        {
          tf2::Vector3 p_frame(pose.pose.position.x, pose.pose.position.y, 0);
          auto p_local = transform_tf2 * p_frame;
          auto dx = p_local.x() - x;
          auto dy = p_local.y() - y;
          auto dist = sqrt(dx * dx + dy * dy);
          if (dist < min_path_dist)
          {
            min_path_dist = dist;
          }
        }
        if (min_path_dist > social_distance_y_)
        {
          speed_limit = std::min(speed_limit, max_v(std::max(0.0, dist - social_distance_y_), max_acc_, delay_));
        }
        else
        {
          speed_limit = std::min(speed_limit, max_v(std::max(0.0, dist - social_distance_x_), max_acc_, delay_));
        }

        ROS_INFO("PeopleSpeedControl people_limit %s dist from path=%.2f x=%.2f y=%.2f vx=%.2f vy=%.2f pt=%.2f sdx=%.2f sdy=%.2f dist=%.2f limit=%.2f",
                 it->name.c_str(), min_path_dist, x, y, vx, vy, pt, sdx, sdy, dist, speed_limit);

        if (speed_limit < max_speed_)
        {
          std_msgs::String msg;

          if (fabs(speed_limit) < 0.01)
          {
            msg.data = "navigation;event;people_speed_stopped";
          }
          else if (vx > 0.25 && speed_limit < max_speed_ * 0.75)
          {
            msg.data = "navigation;event;people_speed_following";
          }

          if (!msg.data.empty())
          {
            ROS_INFO("PeopleSpeedControl %s", msg.data);
            event_pub_.publish(msg);
          }
        }
      }
      lock.unlock();

      std_msgs::Float32 msg;
      msg.data = speed_limit;
      //ROS_INFO("limit = %.2f", speed_limit);
      limit_pub_.publish(msg);
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr &input)
    {
      last_odom_ = *input;
    }

    void planCallback(const nav_msgs::Path::ConstPtr &input)
    {
      NODELET_INFO("PeopleSpeedControl got plan");
      last_plan_ = *input;
    }

    void setSocialDistanceCallback(const geometry_msgs::Point::ConstPtr &input)
    {
      boost::mutex::scoped_lock lock(thread_sync_);

      social_distance_x_ = input->x;
      social_distance_y_ = input->y;

      geometry_msgs::Point msg;
      msg.x = social_distance_x_;
      msg.y = social_distance_y_;
      get_social_distance_pub_.publish(msg);

      NODELET_INFO("PeopleSpeedControl setSocialDistanceCallback social_distance=(%.2f, %.2f)", social_distance_x_, social_distance_y_);
      lock.unlock();
    }

  }; // class PeopleSpeedControlNodelet

  PLUGINLIB_EXPORT_CLASS(Safety::PeopleSpeedControlNodelet, nodelet::Nodelet)
} // namespace Safety
