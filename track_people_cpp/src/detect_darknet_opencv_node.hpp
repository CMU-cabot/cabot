#ifndef DETECT_DARKNET_OPENCV_NODE_HPP
#define DETECT_DARKNET_OPENCV_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include "detect_darknet_opencv.hpp"


namespace TrackPeopleCPP
{
class DetectDarknetOpencvNode : public rclcpp::Node
{
public:
  DetectDarknetOpencvNode(rclcpp::NodeOptions options);
  ~DetectDarknetOpencvNode();

private:
  DetectDarknetOpencv* impl;
}; // class DetectDarknetOpencvNode


} // namespace TrackPeopleCPP


#endif