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

#include <cv_bridge/cv_bridge.h>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <diagnostic_updater/publisher.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <open3d/camera/PinholeCameraIntrinsic.h>
#include <open3d/geometry/Image.h>
#include <open3d/geometry/PointCloud.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/node.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <track_people_py/msg/bounding_box.hpp>
#include <track_people_py/msg/tracked_box.hpp>
#include <track_people_py/msg/tracked_boxes.hpp>

#include <mutex>
#include <opencv2/dnn.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include <queue>


namespace TrackPeopleCPP {
struct DetectData {
  std_msgs::msg::Header header;
  sensor_msgs::msg::Image::ConstPtr rgb_msg_ptr;
  sensor_msgs::msg::Image::ConstPtr depth_msg_ptr;
  // cv_bridge::CvImageConstPtr cv_rgb_ptr;
  // cv_bridge::CvImageConstPtr cv_depth_ptr;
  geometry_msgs::msg::TransformStamped transformStamped;
  int rotate;
  track_people_py::msg::TrackedBoxes result;
  // Pose getPose();
};

class DetectDarknetOpencv {
 public:
  DetectDarknetOpencv();
  void onInit(rclcpp::Node * ptr);

 private:
  void enable_detect_people_cb(const std_srvs::srv::SetBool::Request::SharedPtr req,
                               std_srvs::srv::SetBool::Response::SharedPtr res);
  void camera_info_cb(const sensor_msgs::msg::CameraInfo::SharedPtr info);
  void rgb_depth_img_cb(const sensor_msgs::msg::Image::SharedPtr & rgb_msg_ptr,
                        const sensor_msgs::msg::Image::SharedPtr & depth_msg_ptr);
  void fps_loop_cb();
  void detect_loop_cb();
  void process_detect(DetectData &dd);
  void depth_loop_cb();
  void process_depth(DetectData &dd);
  std::shared_ptr<open3d::geometry::PointCloud> generatePointCloudFromDepthAndBox(DetectData &dd,
                                                                                  track_people_py::msg::BoundingBox &box);
  Eigen::Vector3d getMedianOfPoints(open3d::geometry::PointCloud &pc);

  bool debug_;
  bool parallel_;
  bool is_ready_;

  rclcpp::Node *nh_;

  cv::dnn::Net darknet_;

  std::shared_ptr<cv::dnn::DetectionModel> model_;
  std::shared_ptr<DetectData> temp_dd_;
  std::queue<DetectData> queue_camera_;
  std::queue<DetectData> queue_ready_;
  std::queue<DetectData> queue_detect_;
  int queue_size_;
  std::mutex queue_camera_mutex_;
  std::mutex queue_ready_mutex_;
  std::mutex queue_detect_mutex_;

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
  tf2_ros::Buffer *tfBuffer;

  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr toggle_srv_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Publisher<track_people_py::msg::TrackedBoxes>::SharedPtr detected_boxes_pub_;
  rclcpp::TimerBase::SharedPtr fps_loop_;
  rclcpp::TimerBase::SharedPtr detect_loop_;
  rclcpp::TimerBase::SharedPtr depth_loop_;

  message_filters::Subscriber<sensor_msgs::msg::Image> *rgb_image_sub_;
  message_filters::Subscriber<sensor_msgs::msg::Image> *depth_image_sub_;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image> SyncPolicy;
  message_filters::Synchronizer<SyncPolicy> *rgb_depth_img_synch_;

  diagnostic_updater::Updater *updater_;
  diagnostic_updater::HeaderlessTopicDiagnostic *people_freq_;
  diagnostic_updater::HeaderlessTopicDiagnostic *camera_freq_;
};

}  // namespace TrackPeopleCPP

#endif
