// Copyright (c) 2021  Carnegie Mellon University
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

#ifndef DETECT_DARKNET_OPENCV_HPP
#define DETECT_DARKNET_OPENCV_HPP

#include <thread>
#include <mutex>

#include <ros/ros.h>
#include <std_srvs/SetBool.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>

#include <track_people_py/TrackedBox.h>
#include <track_people_py/TrackedBoxes.h>
#include <track_people_py/BoundingBox.h>

#include <tf2_ros/transform_listener.h>
#include <tf/LinearMath/Transform.h>
#include <tf/transform_datatypes.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>

#include <open3d/geometry/Image.h>
#include <open3d/geometry/PointCloud.h>
#include <open3d/camera/PinholeCameraIntrinsic.h>

namespace TrackPeopleCPP
{
  struct DetectData {
    std_msgs::Header header;
    sensor_msgs::ImageConstPtr rgb_msg_ptr;
    sensor_msgs::ImageConstPtr depth_msg_ptr;
    //cv_bridge::CvImageConstPtr cv_rgb_ptr;
    //cv_bridge::CvImageConstPtr cv_depth_ptr;
    geometry_msgs::TransformStamped transformStamped;
    int rotate;
    track_people_py::TrackedBoxes result;
    //Pose getPose();
  };
    
    
  class DetectDarknetOpencv
  {
  public:
    DetectDarknetOpencv();
    void onInit(ros::NodeHandle &nh);

  private:
    bool enable_detect_people_cb(std_srvs::SetBool::Request &req,
				 std_srvs::SetBool::Response &res);
    void camera_info_cb(const sensor_msgs::CameraInfoConstPtr& info);
    void rgb_depth_img_cb(const sensor_msgs::ImageConstPtr& rgb_img_ptr, const sensor_msgs::ImageConstPtr& depth_img_ptr);
    void fps_loop_cb(const ros::TimerEvent& event);
    void detect_loop_cb(const ros::TimerEvent& event);
    void process_detect(DetectData &dd);
    void depth_loop_cb(const ros::TimerEvent& event);
    void process_depth(DetectData &dd);
    std::shared_ptr<open3d::geometry::PointCloud>
    generatePointCloudFromDepthAndBox(DetectData &dd, track_people_py::BoundingBox& box);
    Eigen::Vector3d getMedianOfPoints(open3d::geometry::PointCloud &pc);
    
    bool debug_;
    bool parallel_;
    bool is_ready_;
    
    cv::dnn::Net darknet_;

    std::shared_ptr<cv::dnn::DetectionModel> model_;
    std::shared_ptr<DetectData> temp_dd_;
    std::queue<DetectData> queue_ready_;
    std::queue<DetectData> queue_detect_;
    int queue_size_;
    std::mutex queue_mutex_;

    bool enable_detect_people_;

    // config parameters
    double detection_threshold_;
    double minimum_detection_size_threshold_;
    std::string detect_config_filename_;
    std::string detect_weight_filename_;
    std::string detect_label_filename_;
    std::string map_frame_name_;
    std::string camera_id_;
    std::string camera_link_frame_name_;
    std::string camera_info_topic_name_;
    std::string image_rect_topic_name_;
    std::string depth_registered_topic_name_;
    bool depth_unit_meter_;
    double target_fps_;

    // image config
    int image_width_;
    int image_height_;
    double focal_length_;
    double center_x_;
    double center_y_;

    std::chrono::time_point<std::chrono::high_resolution_clock> fps_time_;
    int fps_count_;
    double detect_time_;
    int detect_count_;
    double depth_time_;
    int depth_count_;
    
    tf2_ros::TransformListener *tfListener;
    tf2_ros::Buffer tfBuffer;

    ros::ServiceServer toggle_srv_;
    ros::Subscriber camera_info_sub_;
    ros::Publisher detected_boxes_pub_;
    ros::Timer fps_loop_;
    ros::Timer detect_loop_;
    ros::Timer depth_loop_;

    std::thread detect_thread_;
    std::thread depth_thread_;

    message_filters::Subscriber<sensor_msgs::Image>* rgb_image_sub_;
    message_filters::Subscriber<sensor_msgs::Image>* depth_image_sub_;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> SyncPolicy;
    message_filters::Synchronizer<SyncPolicy> *rgb_depth_img_synch_;
    
  };
  
} // namespace TrackPeopleCPP

#endif

