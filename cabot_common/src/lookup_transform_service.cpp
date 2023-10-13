// Copyright (c) 2023  Carnegie Mellon University
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
// lookup transform service
// Author: Daisuke Sato <daisukes@cmu.edu>

#include "rclcpp/rclcpp.hpp"
#include "cabot_msgs/srv/lookup_transform.hpp"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include <memory>


namespace CaBot
{
class LookupTransformServiceNode : public rclcpp::Node
{
 public:
  explicit LookupTransformServiceNode(const rclcpp::NodeOptions & options)
      : rclcpp::Node("lookup_transform_service", options),
        buffer_(this->get_clock(), tf2::durationFromSec(10)),
        listener_(buffer_)
  {
    service_ = this->create_service<cabot_msgs::srv::LookupTransform>(
        "lookup_transform",
        std::bind(&LookupTransformServiceNode::lookupTransform, this, std::placeholders::_1, std::placeholders::_2));
    RCLCPP_INFO(rclcpp::get_logger("lookup_transform"), "Ready to lookup transform");
  }

  void lookupTransform(const std::shared_ptr<cabot_msgs::srv::LookupTransform::Request> request,
                        std::shared_ptr<cabot_msgs::srv::LookupTransform::Response> response)
  {
    try {
      response->transform = buffer_.lookupTransform(request->target_frame, request->source_frame, tf2_ros::fromMsg(request->time));
    } catch (const tf2::ConnectivityException & ex) {
      response->error.error = response->error.CONNECTIVITY_ERROR;
      response->error.error_string = ex.what();
    } catch (const tf2::LookupException & ex) {
      response->error.error = response->error.LOOKUP_ERROR;
      response->error.error_string = ex.what();
    } catch (const tf2::ExtrapolationException & ex) {
      response->error.error = response->error.EXTRAPOLATION_ERROR;
      response->error.error_string = ex.what();
    } catch (const tf2::InvalidArgumentException & ex) {
      response->error.error = response->error.INVALID_ARGUMENT_ERROR;
      response->error.error_string = ex.what();
    } catch (const tf2::TimeoutException & ex) {
      response->error.error = response->error.TIMEOUT_ERROR;
      response->error.error_string = ex.what();
    } catch (const tf2::TransformException & ex) {
      response->error.error = response->error.TRANSFORM_ERROR;
      response->error.error_string = ex.what();
    }
  }

 private:
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener listener_;
  rclcpp::Service<cabot_msgs::srv::LookupTransform>::SharedPtr service_;

};  // class LookupTransformServiceNode

}  // namespace CaBot
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(CaBot::LookupTransformServiceNode)
