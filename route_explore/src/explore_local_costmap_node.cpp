// Copyright (c) 2023  Carnegie Mellon University, IBM Corporation, and others
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

#include <mutex>
#include <thread>

#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/utils.h>

namespace cabot_explore
{
    class LocalCostmap
    {
    public:
        LocalCostmap(ros::NodeHandle* nodehandle):nh_(*nodehandle),
                                                    costmap_topic_("map"),
                                                    local_costmap_topic_("/cabot_explore/local_map"),
                                                    map_frame_("map"),
                                                    robot_base_frame_("base_footprint"),
                                                    max_local_map_width_(30.0),
                                                    max_local_map_height_(30.0)
        {
            ROS_INFO("LocalCostmap Constructor");

            nh_.getParam("costmap_topic", costmap_topic_);
            nh_.getParam("local_costmap_topic", local_costmap_topic_);
            nh_.getParam("map_frame", map_frame_);
            nh_.getParam("robot_base_frame", robot_base_frame_);
            nh_.getParam("max_local_map_width", max_local_map_width_);
            nh_.getParam("max_local_map_height", max_local_map_height_);

            last_map_msg_time_ = ros::Time(0);

            tfListener_ = new tf2_ros::TransformListener(tfBuffer_);

            map_subscriber_ = nh_.subscribe(costmap_topic_, 1, &LocalCostmap::mapCallback, this);
            local_map_publisher_ = nh_.advertise<nav_msgs::OccupancyGrid>(local_costmap_topic_, 1);
        }

        ~LocalCostmap()
        {
            ROS_INFO("LocalCostmap Destructor");
        }

        void spin()
        {
            std::thread local_costmap_thread([this]() {
                while (nh_.ok()) {
                    extractLocalMap();
                    std::this_thread::sleep_for(std::chrono::milliseconds(1));
                }
            });
            ros::spin();
            local_costmap_thread.join();
        }

    private:
        void mapCallback(const nav_msgs::OccupancyGridConstPtr & msg)
        {
            // ROS_INFO("Received occupancy grid");
            std::lock_guard<std::mutex> lock(map_msg_mutex_);
            map_msg_ptr_ = msg;
        }

        void extractLocalMap()
        {
            nav_msgs::OccupancyGridConstPtr map_msg;
            {
                std::lock_guard<std::mutex> lock(map_msg_mutex_);
                map_msg = map_msg_ptr_;
            }

            if (map_msg==NULL || map_msg->header.stamp <= last_map_msg_time_) {
                return;
            }

            tf2::Transform robot_pose(tf2::Quaternion::getIdentity(), tf2::Vector3(0, 0, 0));
            geometry_msgs::TransformStamped map_to_robot_msg;
            try {
                map_to_robot_msg = tfBuffer_.lookupTransform(map_frame_, robot_base_frame_, ros::Time(0), ros::Duration(1.0));
            } catch (tf2::TransformException &ex) {
                ROS_INFO("%s", ex.what());
                return;
            }
            tf2::Stamped<tf2::Transform> map_to_robot_tf2;
            tf2::fromMsg(map_to_robot_msg, map_to_robot_tf2);
            robot_pose *= map_to_robot_tf2;
            // ROS_INFO("Robot position = (%.2f,%.2f), yaw = %.2f", robot_pose.getOrigin().x(), robot_pose.getOrigin().y(), tf2::getYaw(robot_pose.getRotation()));

            nav_msgs::OccupancyGrid local_map;
            local_map.header.stamp = map_msg->header.stamp;
            local_map.header.frame_id = map_msg->header.frame_id;
            local_map.info.map_load_time = map_msg->info.map_load_time;
            local_map.info.resolution = map_msg->info.resolution;
            local_map.info.width = (unsigned int)(max_local_map_width_/local_map.info.resolution);
            local_map.info.height = (unsigned int)(max_local_map_height_/local_map.info.resolution);
            local_map.info.origin = map_msg->info.origin;
            local_map.info.origin.position.x = robot_pose.getOrigin().x() - (local_map.info.width/2.0)*local_map.info.resolution;
            local_map.info.origin.position.y = robot_pose.getOrigin().y() - (local_map.info.height/2.0)*local_map.info.resolution;

            int map_offset_x = (local_map.info.origin.position.x - map_msg->info.origin.position.x)/map_msg->info.resolution;
            int map_offset_y = (local_map.info.origin.position.y - map_msg->info.origin.position.y)/map_msg->info.resolution;

            // copy map data
            local_map.data.assign(local_map.info.height*local_map.info.width, -1);
            for (int y = 0; y < local_map.info.height; y++) {
                for (int x = 0; x < local_map.info.width; x++) {
                    int map_y = y + map_offset_y;
                    int map_x = x + map_offset_x;
                    if (map_y>=0 && map_x>=0 && map_y<map_msg->info.height && map_x<map_msg->info.width) {
                        local_map.data.at(y*local_map.info.width + x) = map_msg->data.at(map_y*map_msg->info.width + map_x);
                    }
                }
            }
            local_map_publisher_.publish(local_map);
            last_map_msg_time_ = map_msg->header.stamp;
            // ROS_INFO("Published local map");
        }

        std::string costmap_topic_;
        std::string local_costmap_topic_;
        std::string map_frame_;
        std::string robot_base_frame_;
        float max_local_map_width_;
        float max_local_map_height_;

        std::mutex map_msg_mutex_;
        nav_msgs::OccupancyGridConstPtr map_msg_ptr_;
        ros::Time last_map_msg_time_;

        ros::NodeHandle nh_;
        tf2_ros::TransformListener *tfListener_;
        tf2_ros::Buffer tfBuffer_;
        ros::Subscriber map_subscriber_;
        ros::Publisher local_map_publisher_;
    };
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "local_costmap_node");
    ros::NodeHandle nh("~");
    cabot_explore::LocalCostmap localCostmap(&nh);
    localCostmap.spin();
}