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

#include "detect_darknet_opencv.hpp"

namespace TrackPeopleCPP {
DetectDarknetOpencv::DetectDarknetOpencv()
    : enable_detect_people_(true),
      queue_size_(2),
      debug_(false),
      parallel_(true),
      fps_count_(0),
      detect_time_(0),
      detect_count_(0),
      depth_time_(0),
      depth_count_(0),
      is_ready_(false),
      people_freq_(NULL),
      camera_freq_(NULL) {
  if (debug_) {
    cv::namedWindow("Depth", cv::WINDOW_AUTOSIZE);
  }
}

void DetectDarknetOpencv::onInit(ros::NodeHandle &nh) {
  nh.getParam("track_people_py/detection_threshold", detection_threshold_);
  // minimum vertical size of box to consider a detection as a track
  nh.getParam("track_people_py/minimum_detection_size_threshold", minimum_detection_size_threshold_);

  nh.getParam("track_people_py/detect_config_file", detect_config_filename_);
  nh.getParam("track_people_py/detect_weight_file", detect_weight_filename_);
  // nh.getParam("track_people_py/detect_label_file", detect_label_filename_);

  ROS_INFO("weights: %s", detect_weight_filename_.c_str());
  ROS_INFO("config : %s", detect_config_filename_.c_str());
  darknet_ = cv::dnn::readNet(detect_weight_filename_, detect_config_filename_, "Darknet");
  darknet_.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
  darknet_.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA_FP16);

  model_ = std::make_shared<cv::dnn::DetectionModel>(cv::dnn::DetectionModel(darknet_));
  model_->setInputParams(1 / 255.0, cv::Size(416, 416), cv::Scalar(), true);

  cv::Mat dummy = cv::Mat::zeros(cv::Size(1280, 720), CV_8UC3);
  std::vector<int> classIds;
  std::vector<float> scores;
  std::vector<cv::Rect> boxes;
  model_->detect(dummy, classIds, scores, boxes, 0.6, 0.4);
  ROS_INFO("Model Loaded");

  nh.getParam("track_people_py/map_frame", map_frame_name_);
  nh.getParam("track_people_py/camera_id", camera_id_);
  nh.getParam("track_people_py/camera_link_frame", camera_link_frame_name_);
  nh.getParam("track_people_py/camera_info_topic", camera_info_topic_name_);
  nh.getParam("track_people_py/image_rect_topic", image_rect_topic_name_);
  nh.getParam("track_people_py/depth_registered_topic", depth_registered_topic_name_);
  nh.getParam("track_people_py/depth_unit_meter", depth_unit_meter_);
  nh.getParam("track_people_py/target_fps", target_fps_);

  toggle_srv_ = nh.advertiseService("enable_detect_people", &DetectDarknetOpencv::enable_detect_people_cb, this);

  // ROS_INFO("Waiting for camera_info topic...");
  // ros::topic::waitForMessage<sensor_msgs::CameraInfo>(camera_info_topic_name_, nh);
  // ROS_INFO("Found camera_info topic.");

  camera_info_sub_ = nh.subscribe(camera_info_topic_name_, 10, &DetectDarknetOpencv::camera_info_cb, this);

  tfListener = new tf2_ros::TransformListener(tfBuffer);

  ROS_INFO("subscribe to %s and %s", image_rect_topic_name_.c_str(), depth_registered_topic_name_.c_str());
  rgb_image_sub_ = new message_filters::Subscriber<sensor_msgs::Image>(nh, image_rect_topic_name_, 10);
  depth_image_sub_ = new message_filters::Subscriber<sensor_msgs::Image>(nh, depth_registered_topic_name_, 10);
  rgb_depth_img_synch_ =
      new message_filters::Synchronizer<SyncPolicy>(SyncPolicy(10), *rgb_image_sub_, *depth_image_sub_);
  rgb_depth_img_synch_->registerCallback(&DetectDarknetOpencv::rgb_depth_img_cb, this);

  detected_boxes_pub_ = nh.advertise<track_people_py::TrackedBoxes>("/track_people_py/detected_boxes", 1);

  loop_thread_ = std::thread(
      [](DetectDarknetOpencv *obj) {
        ros::TimerEvent event;
        std::chrono::time_point<std::chrono::high_resolution_clock> last = std::chrono::high_resolution_clock::now();
        while (ros::ok()) {
          auto now = std::chrono::high_resolution_clock::now();
          auto diff = ((double)(now - last).count()) / 1000000000;
          if (obj->is_ready_ && diff > 1.0 / obj->target_fps_) {
            last = std::chrono::high_resolution_clock::now();
            obj->fps_loop_cb(event);
          }
          std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
      },
      this);
  // fps_loop_ = nh.createTimer(ros::Duration(1.0 / target_fps_), &DetectDarknetOpencv::fps_loop_cb, this);
  // detect_loop_ = nh.createTimer(ros::Duration(0.01), &DetectDarknetOpencv::detect_loop_cb, this);
  // depth_loop_ = nh.createTimer(ros::Duration(0.01), &DetectDarknetOpencv::depth_loop_cb, this);

  detect_thread_ = std::thread(
      [](DetectDarknetOpencv *obj) {
        ros::TimerEvent event;
        while (ros::ok()) {
          if (obj->is_ready_) {
            obj->detect_loop_cb(event);
          }
          std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
      },
      this);

  depth_thread_ = std::thread(
      [](DetectDarknetOpencv *obj) {
        ros::TimerEvent event;
        while (ros::ok()) {
          if (obj->is_ready_) {
            obj->depth_loop_cb(event);
          }
          std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
      },
      this);

  updater_.setHardwareID(nh.getNamespace());
  diagnostic_updater::FrequencyStatusParam param1(&target_fps_, &target_fps_, 1.0, 2);
  camera_freq_ = new diagnostic_updater::HeaderlessTopicDiagnostic("CameraInput", updater_, param1);
  diagnostic_updater::FrequencyStatusParam param2(&target_fps_, &target_fps_, 0.2, 2);
  people_freq_ = new diagnostic_updater::HeaderlessTopicDiagnostic("PeopleDetect", updater_, param2);
}

bool DetectDarknetOpencv::enable_detect_people_cb(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res) {}

void DetectDarknetOpencv::camera_info_cb(const sensor_msgs::CameraInfoConstPtr &info) {
  ROS_INFO("Got camera_info topic.");
  image_width_ = info->width;
  image_height_ = info->height;
  focal_length_ = info->K[0];
  center_x_ = info->K[2];
  center_y_ = info->K[5];
  camera_info_sub_.shutdown();
  ROS_INFO("Found camera_info topic.");
  is_ready_ = true;
}

geometry_msgs::Pose transformStamped2pose(geometry_msgs::TransformStamped t) {
  geometry_msgs::Pose camera_link_pose;
  camera_link_pose.position.x = t.transform.translation.x;
  camera_link_pose.position.y = t.transform.translation.y;
  camera_link_pose.position.z = t.transform.translation.z;
  camera_link_pose.orientation.x = t.transform.rotation.x;
  camera_link_pose.orientation.y = t.transform.rotation.y;
  camera_link_pose.orientation.z = t.transform.rotation.z;
  camera_link_pose.orientation.w = t.transform.rotation.w;
  return camera_link_pose;
}

void DetectDarknetOpencv::rgb_depth_img_cb(const sensor_msgs::ImageConstPtr &rgb_msg_ptr,
                                           const sensor_msgs::ImageConstPtr &depth_msg_ptr) {
  try {
    DetectData dd;
    dd.header = rgb_msg_ptr->header;
    dd.transformStamped = tfBuffer.lookupTransform(map_frame_name_, camera_link_frame_name_, rgb_msg_ptr->header.stamp,
                                                   ros::Duration(1.0));

    // deal with rotation
    tf::Transform pose_tf;
    tf::transformMsgToTF(dd.transformStamped.transform, pose_tf);
    tf::Matrix3x3 mat(pose_tf.getRotation());
    double yaw, pitch, roll;
    mat.getRPY(roll, pitch, yaw);
    if (-M_PI / 4 < roll && roll < M_PI / 4) {
      dd.rotate = 0;
    } else if (M_PI / 4 <= roll && roll < M_PI / 4 * 3) {  // rotate 90 degree
      dd.rotate = 1;
    } else if (M_PI / 4 * 3 <= roll || roll <= -M_PI / 4 * 3) {  // up side down
      dd.rotate = 2;
    } else if (-M_PI / 4 * 3 < roll && roll <= -M_PI / 4) {  // rotate -90 degree
      dd.rotate = 3;
    }

    dd.rgb_msg_ptr = rgb_msg_ptr;
    dd.depth_msg_ptr = depth_msg_ptr;

    camera_freq_->tick();
    fps_count_++;
    temp_dd_ = std::make_shared<DetectData>(dd);

    {
      std::lock_guard<std::mutex> lock(queue_camera_mutex_);
      if (queue_camera_.size() < queue_size_) {
        queue_camera_.push(*temp_dd_);
      } else {
        queue_camera_.pop();
        queue_camera_.push(*temp_dd_);
      }
    }

    if (debug_) {
      int MAX = 100;

      if (fps_count_ == 0) {
        fps_time_ = std::chrono::high_resolution_clock::now();
      }
      if (fps_count_ == MAX) {
        auto now = std::chrono::high_resolution_clock::now();
        auto diff = ((double)(now - fps_time_).count()) / 1000000000;
        ROS_INFO("fps %.2f (%.2f, %d)", fps_count_ / diff, diff, fps_count_);
        fps_count_ = 0;
        fps_time_ = std::chrono::high_resolution_clock::now();
      }

      if (detect_count_ == MAX) {
        ROS_INFO("detect %.2f fps %d", detect_count_ / detect_time_, detect_count_);
        detect_time_ = 0;
        detect_count_ = 0;

        ROS_INFO("depth %.2f fps %d", depth_count_ / depth_time_, depth_count_);
        depth_time_ = 0;
        depth_count_ = 0;
      }
    }
  } catch (std::exception &e) {
    ROS_INFO("tf2 error: %s", e.what());
  }
}

void DetectDarknetOpencv::fps_loop_cb(const ros::TimerEvent &event) {
  updater_.update();
  if (queue_ready_.size() == queue_size_) {
    return;
  }
  if (parallel_) {
    DetectData dd;
    {
      std::lock_guard<std::mutex> lock(queue_camera_mutex_);
      if (queue_camera_.size() == 0) {
        return;
      }
      dd = queue_camera_.front();
      queue_camera_.pop();
    }
    {
      std::lock_guard<std::mutex> lock(queue_ready_mutex_);
      queue_ready_.push(dd);
    }
  } else {
    if (people_freq_ != NULL) {
      people_freq_->tick();
    }
    DetectData dd = *temp_dd_;
    process_detect(dd);
    process_depth(dd);
    detected_boxes_pub_.publish(dd.result);
    temp_dd_ = nullptr;
  }
}

void DetectDarknetOpencv::detect_loop_cb(const ros::TimerEvent &event) {
  DetectData dd;
  {
    std::lock_guard<std::mutex> lock(queue_ready_mutex_);
    if (queue_ready_.size() == 0) {
      return;
    }
    dd= queue_ready_.front();
    queue_ready_.pop();
  }

  auto start = std::chrono::high_resolution_clock::now();
  process_detect(dd);
  auto end = std::chrono::high_resolution_clock::now();
  detect_time_ += ((double)(end - start).count()) / 1000000000;
  detect_count_++;

  std::lock_guard<std::mutex> lock(queue_detect_mutex_);
  queue_detect_.push(dd);
}

void DetectDarknetOpencv::depth_loop_cb(const ros::TimerEvent &event) {
  DetectData dd;
  {
    std::lock_guard<std::mutex> lock(queue_detect_mutex_);
    if (queue_detect_.size() == 0) {
      return;
    }
    dd = queue_detect_.front();
    queue_detect_.pop();
  }
  if (people_freq_ != NULL) {
    people_freq_->tick();
  }
  auto start = std::chrono::high_resolution_clock::now();
  process_depth(dd);
  detected_boxes_pub_.publish(dd.result);
  auto end = std::chrono::high_resolution_clock::now();
  depth_time_ += ((double)(end - start).count()) / 1000000000;
  depth_count_++;
}

void DetectDarknetOpencv::process_detect(DetectData &dd) {
  auto cv_rgb_ptr = cv_bridge::toCvShare(dd.rgb_msg_ptr, sensor_msgs::image_encodings::BGR8);
  const cv::Mat &img = cv_rgb_ptr->image;

  cv::Mat rImg;
  if (dd.rotate == 0) {
    rImg = img;
  } else {
    // rotate
    cv::rotate(img, rImg, dd.rotate - 1);
  }

  static std_msgs::ColorRGBA red;
  red.r = 1.0;

  std::vector<int> classIds;
  std::vector<float> scores;
  std::vector<cv::Rect> boxes;
  model_->detect(rImg, classIds, scores, boxes, 0.6, 0.4);

  track_people_py::TrackedBoxes &tbs = dd.result;
  tbs.header = dd.header;
  tbs.header.frame_id = map_frame_name_;
  tbs.camera_id = camera_id_;
  // tbs.pose = dd.getPose();
  for (int i = 0; i < classIds.size(); i++) {
    auto classId = classIds[i];
    auto score = scores[i];
    auto box = boxes[i];

    if (classId != 0 || score < detection_threshold_ || box.width < minimum_detection_size_threshold_ ||
        box.height < minimum_detection_size_threshold_) {
      continue;
    }

    // rotate back the detected box coordinate
    if (dd.rotate > 0) {
      cv::Size s = rImg.size();
      int x, y, w, h;
      if (dd.rotate == 1) {
        x = box.y;
        y = s.width - box.x - box.width;
        w = box.height;
        h = box.width;
      } else if (dd.rotate == 2) {
        x = s.width - box.x - box.width;
        y = s.height - box.y - box.height;
        w = box.width;
        h = box.height;
      } else if (dd.rotate == 3) {
        x = s.height - box.y - box.height;
        y = box.x;
        w = box.height;
        h = box.width;
      }

      box.x = x;
      box.y = y;
      box.width = w;
      box.height = h;
    }

    track_people_py::TrackedBox tb;
    tb.header = dd.header;
    tb.header.frame_id = map_frame_name_;
    tb.color = red;
    tb.box.Class = "person";
    tb.box.probability = score;
    tb.box.xmin = box.x;
    tb.box.ymin = box.y;
    tb.box.xmax = box.x + box.width;
    tb.box.ymax = box.y + box.height;
    tb.center3d.x = 1;
    tb.center3d.y = 1;
    tb.center3d.z = 1;
    tbs.tracked_boxes.push_back(tb);

    if (debug_) {
      rectangle(img, box, cv::Scalar(0, 255, 0), 2);  // draw rectangle
      cv::imshow("Depth", img);
      cv::waitKey(100);
    }
  }
}

void DetectDarknetOpencv::process_depth(DetectData &dd) {
  std::vector<track_people_py::TrackedBox> &tracked_boxes = dd.result.tracked_boxes;
  tf::Transform pose_tf;
  tf::transformMsgToTF(dd.transformStamped.transform, pose_tf);

  if (tracked_boxes.size() == 0) {
    return;
  }

  for (auto it = tracked_boxes.begin(); it != tracked_boxes.end();) {
    auto pc = generatePointCloudFromDepthAndBox(dd, it->box);
    if (!pc->HasPoints()) {
      ROS_INFO("no points found");
      tracked_boxes.erase(it);
      continue;
    }

    auto median = getMedianOfPoints(*pc);

    if (median.hasNaN()) {
      ROS_INFO("median has NAN");
      tracked_boxes.erase(it);
      continue;
    }

    // convert realsense coordinate (x:left-right, y:top-down,   z:back-front) to
    //               ROS coordinate (x:back-front, y:right-left, z:down-top)
    double x = median(2);
    double y = -median(0);
    double z = 0;  // do not use height

    // if camera is sideway: roll = 90 or 270 degree
    if (dd.rotate % 2 == 1) {
      y = 0;
      z = -median(1);
    }

    tf::Transform median_tf(tf::Quaternion(0, 0, 0, 1), tf::Vector3(x, y, z));
    auto map_center = (pose_tf * median_tf).getOrigin();

    it->center3d.x = map_center.x();
    it->center3d.y = map_center.y();
    it->center3d.z = map_center.z();
    it++;
  }
}

std::shared_ptr<open3d::geometry::PointCloud> DetectDarknetOpencv::generatePointCloudFromDepthAndBox(
    DetectData &dd, track_people_py::BoundingBox &box) {
  auto o3d_depth_ptr = std::make_shared<open3d::geometry::Image>();

  // ROS_INFO("depth_msg_ptr.use_count() %d", dd.depth_msg_ptr.use_count());
  auto cv_depth_ptr = cv_bridge::toCvShare(dd.depth_msg_ptr, sensor_msgs::image_encodings::TYPE_16UC1);
  // ROS_INFO("cv_depth_ptr.use_count() %d", cv_depth_ptr.use_count());
  auto img = cv_depth_ptr->image;
  // ROS_INFO("img %d %d", img.cols, img.rows);

  o3d_depth_ptr->Prepare(img.cols, img.rows, 1 /* channel */, 4 /* bytes per channel */);  // convert to float image
  fill(o3d_depth_ptr->data_.begin(), o3d_depth_ptr->data_.end(), 0);

  for (int row = box.ymin; row < box.ymax; row++) {
    for (int col = box.xmin; col < box.xmax; col++) {
      auto p = o3d_depth_ptr->PointerAt<float>(col, row);
      *p = ((float)img.at<uint16_t>(row, col)) / 1000;
    }
  }
  /*
  for (int row = 0; row < img.rows; row++) {
    for (int col = 0; col < img.cols; col++) {
      if (box.ymin <= row && row <= box.ymax &&
          box.xmin <= col && col <= box.xmax) {
        auto p = o3d_depth_ptr->PointerAt<float>(col, row);
        *p = ((float)img.at<uint16_t>(row, col)) / 1000;

        img.at<uint16_t>(row, col) *= 10;
      } else {
        auto p = o3d_depth_ptr->PointerAt<float>(col, row);
        *p = 0;
        img.at<uint16_t>(row, col) = 0;
      }
    }
  }
  */

  if (debug_) {
    cv::imshow("Depth", img);
    cv::waitKey(100);
  }

  // ROS_INFO("%d %d %.2f %.2f %.2f %.2f", image_width_, image_height_, focal_length_, focal_length_, center_x_,
  // center_y_);
  auto pinhole_camera_intrinsic = open3d::camera::PinholeCameraIntrinsic(image_width_, image_height_, focal_length_,
                                                                         focal_length_, center_x_, center_y_);

  return open3d::geometry::PointCloud::CreateFromDepthImage(*o3d_depth_ptr, pinhole_camera_intrinsic);
}

Eigen::Vector3d DetectDarknetOpencv::getMedianOfPoints(open3d::geometry::PointCloud &pc) {
  Eigen::Vector3d ret;

  auto &ps = pc.points_;
  for (int i = 0; i < 3; i++) {
    // partially sort until median, 50 percentile
    std::nth_element(ps.begin(), ps.begin() + ps.size() / 2, ps.end(),
                     [i](Eigen::Vector3d &a, Eigen::Vector3d &b) { return a(i) < b(i); });
    ret[i] = ps[ps.size() / 2][i];
  }

  return ret;
}

}  // namespace TrackPeopleCPP
