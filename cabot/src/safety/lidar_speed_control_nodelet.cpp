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

// LiDAR speed control
// Author: Daisuke Sato <daisukes@cmu.edu>

#include <limits>
#include <sstream>

#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <ros/time.h>

#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/utils.h>

#include <std_msgs/Float32.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Point.h>

#include "cabot/util.hpp"

namespace Safety
{
	class LiDARSpeedControlNodelet : public nodelet::Nodelet
	{

	public:
		std::string laser_topic_;
		std::string vis_topic_;
		std::string limit_topic_;

		std::string map_frame_;
		std::string robot_base_frame_;

		bool check_blind_space_;
		bool check_front_obstacle_;

		double max_speed_;
		double min_speed_;
	        double max_acc_;
		double limit_factor_;

		ros::Subscriber scan_sub_;
		ros::Publisher vis_pub_;
		ros::Publisher limit_pub_;
		tf2_ros::TransformListener *tfListener;
		tf2_ros::Buffer tfBuffer;
		//message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub_;
		//tf::MessageFilter<sensor_msgs::LaserScan> laser_notifier_;

		LiDARSpeedControlNodelet()
			: laser_topic_("/scan"),
			  vis_topic_("visualize"),
			  limit_topic_("lidar_limit"),
			  map_frame_("map"),
			  robot_base_frame_("base_footprint"),
			  check_blind_space_(true),
			  check_front_obstacle_(true),
			  max_speed_(1.0),
			  min_speed_(0.1),
			  max_acc_(0.6),
			  limit_factor_(3.0)
		{
			NODELET_INFO("LiDARSpeedControlNodeletClass Constructor");
			tfListener = new tf2_ros::TransformListener(tfBuffer);
		}

		~LiDARSpeedControlNodelet()
		{
			NODELET_INFO("LiDARSpeedControlNodeletClass Destructor");
		}

	private:
		void onInit()
		{
			NODELET_INFO("LiDAR speed control - %s", __FUNCTION__);
			ros::NodeHandle &private_nh = getPrivateNodeHandle();

			if (private_nh.hasParam("laser_topic"))
				private_nh.getParam("laser_topic", laser_topic_);

			scan_sub_ = private_nh.subscribe(laser_topic_, 1000, &LiDARSpeedControlNodelet::laserCallback, this);

			private_nh.getParam("visualize_topic", vis_topic_);
			vis_pub_ = private_nh.advertise<visualization_msgs::MarkerArray>(vis_topic_, 100);

			private_nh.getParam("limit_topic", limit_topic_);
			limit_pub_ = private_nh.advertise<std_msgs::Float32>(limit_topic_, 100);

			private_nh.getParam("check_blind_space", check_blind_space_);
			private_nh.getParam("check_front_obstacle", check_front_obstacle_);
			private_nh.getParam("max_speed_", max_speed_);
			private_nh.getParam("min_speed_", min_speed_);
			private_nh.getParam("max_acc_", max_acc_);
			private_nh.getParam("limit_factor_", limit_factor_);

			NODELET_INFO("LiDARSpeedControl with check_blind_space=%s, check_front_obstacle=%s, max_speed=%.2f",
						 check_blind_space_ ? "true" : "false", check_front_obstacle_ ? "true" : "false", max_speed_);
		}

		struct BlindSpot : Safety::Point
		{
			ros::Time last_confirmed;
			int count;
		};

		std::vector<BlindSpot> blind_spots;

		double BLIND_SPOT_MIN_SIZE = 1.0;
		double BLIND_SPOT_MAX_ANGLE = -0.1;
		double BLIND_SPOT_MAX_DISTANCE = 5.0;
		double BLIND_SPOT_MIN_COUNT = 5;

		void laserCallback(const sensor_msgs::LaserScan::ConstPtr input)
		{
			double inf = std::numeric_limits<double>::infinity();
			double speed_limit = max_speed_;

			tf2::Transform robot_pose(tf2::Quaternion::getIdentity(), tf2::Vector3(0, 0, 0));

			// transform map -> base_footprint
			geometry_msgs::TransformStamped map_to_robot_msg;
			try
			{
				map_to_robot_msg = tfBuffer.lookupTransform("map", "base_footprint",
															ros::Time(0), ros::Duration(1.0));
			}
			catch (tf2::TransformException &ex)
			{
				NODELET_WARN("%s", ex.what());
				return;
			}
			tf2::Stamped<tf2::Transform> map_to_robot_tf2;
			tf2::fromMsg(map_to_robot_msg, map_to_robot_tf2);

			robot_pose *= map_to_robot_tf2;

			//NODELET_INFO("%.2f,%.2f,%.2f", robot_pose.getOrigin().x(), robot_pose.getOrigin().y(), tf2::getYaw(robot_pose.getRotation()));

			//clear(vis_pub_);
			Safety::add_point(robot_pose, 0.2, 0, 1, 0, 1);

			geometry_msgs::TransformStamped robot_to_lidar_msg;
			tf2::Stamped<tf2::Transform> robot_to_lidar_tf2;
			try
			{
				robot_to_lidar_msg = tfBuffer.lookupTransform(robot_base_frame_, input->header.frame_id,
															  input->header.stamp, ros::Duration(1.0));
			}
			catch (tf2::TransformException &ex)
			{
				NODELET_WARN("%s", ex.what());
				return;
			}
			tf2::fromMsg(robot_to_lidar_msg, robot_to_lidar_tf2);

			Safety::Line robot(robot_pose);
			//NODELET_INFO("%.2f,%.2f,%.2f,%.2f", robot.s.x, robot.s.y, robot.e.x, robot.e.y);

			if (check_front_obstacle_)
			{ // Check front obstacle (mainly for avoinding from speeding up near the wall
				// get some points in front of the robot

				double range_average = 0;
				int range_count = 0;
				for (int i = 0; i < input->ranges.size(); i++)
				{
					double angle = input->angle_min + input->angle_increment * i;
					if (std::abs(angle) > 0.1)
					{ // if it is not in front of the robot
						continue;
					}

					double range = input->ranges[i];
					if (range == inf)
					{
						range = input->range_max;
					}

					range_average += range;
					range_count += 1;
				}
				range_average /= range_count; // get averge distance of points in front of the robot

				// calculate the speed
				speed_limit = std::min(max_speed_, std::max(min_speed_, (range_average - 0.5) / limit_factor_));
			}

			if (check_blind_space_)
			{ // Check blind space
				Point prev;
				for (int i = 0; i < input->ranges.size(); i++)
				{
					double angle = input->angle_min + input->angle_increment * i;
					double range = input->ranges[i];
					if (range == inf)
					{
						range = input->range_max;
					}

					Point curr(range * cos(angle), range * sin(angle));
					curr.transform(map_to_robot_tf2 * robot_to_lidar_tf2);

					if (prev.x != 0 && prev.y != 0)
					{
						Safety::Line line(prev, curr);
						double len = line.length();
						double cross = robot.cross(line) / robot.length() / line.length();
						double ds = robot.s.distanceTo(prev);
						double de = robot.e.distanceTo(curr);
						Point closest = (ds < de) ? prev : curr;
						double dist = std::min(ds, de);

						if (len > BLIND_SPOT_MIN_SIZE &&
							cross < BLIND_SPOT_MAX_ANGLE &&
							dist < BLIND_SPOT_MAX_DISTANCE)
						{
							// found a spot

							int j = 0;
							for (; j < blind_spots.size(); j++)
							{
								double d = blind_spots[j].distanceTo(closest);
								if (d < 0.3)
								{ // hulostic
									// same spot and update it
									blind_spots[j].count += 1;
									blind_spots[j].x = closest.x;
									blind_spots[j].y = closest.y;
									blind_spots[j].last_confirmed = ros::Time::now();
									break;
								}
							}

							if (j == blind_spots.size())
							{
								// different spot in the list
								BlindSpot bs;
								bs.x = closest.x;
								bs.y = closest.y;
								bs.count = 1;
								bs.last_confirmed = ros::Time::now();
								blind_spots.push_back(bs);
							}
						}
					}
					prev = curr;
				}

				// do something after filter

				for (int i = 0; i < blind_spots.size(); i++)
				{
					BlindSpot bs = blind_spots[i];
					if (bs.count < BLIND_SPOT_MIN_COUNT)
					{
						continue;
					}

					Safety::add_point(bs, 0.2, 1, 0, 0, 1);

					Safety::Point closestToRobot = robot.closestPoint(bs);
					Safety::Line l1(bs, closestToRobot);
					Safety::Line l2(robot.s, closestToRobot);

					// if robot pass the spot, remove it
					if (robot.dot(l2) < 0)
					{
						blind_spots[i].count = 0;
						continue;
					}

					Safety::add_line(l1, 0.05, 1, 0, 0, 1);
					Safety::add_arrow(l2, 0.1, 1, 0, 0, 1);

					// calculate speed limit
					// v = -a*t0 + sqrt(a^2*t0^2+2Da)
					double delay = 0.1;									  // sec (t0)
					double critical_distance = l1.length() + l2.length(); // = 2D
					double limit = -max_acc_ * delay + sqrt(max_acc_ * max_acc_ * delay * delay +
														   critical_distance * max_acc_);
					// update speed limit
					if (limit < speed_limit)
					{
						speed_limit = limit;
					}
				}

				// remove unused spot
				for (int i = blind_spots.size() - 1; i >= 0; i--)
				{
					BlindSpot bs = blind_spots[i];
					double timediff = (ros::Time::now() - bs.last_confirmed).toSec();
					if (timediff > 0.1 * std::max(5, bs.count))
					{
						blind_spots.erase(blind_spots.begin() + i);
					}
				}
			}

			// Publishing the speed limit
			std_msgs::Float32 msg;
			msg.data = speed_limit;
			//NODELET_INFO("limit = %.2f", speed_limit);

			// Publishing the visualization
			char buff[100];
			snprintf(buff, sizeof(buff), "limit - %.2fm/s", speed_limit);
			std::string buffAsStdStr = buff;
			Safety::add_text(buff, robot.s);
			limit_pub_.publish(msg);
			Safety::commit(vis_pub_);
		}

	}; // class LiDARSpeedControlNodelet

	PLUGINLIB_EXPORT_CLASS(Safety::LiDARSpeedControlNodelet, nodelet::Nodelet)
} // namespace Safety
