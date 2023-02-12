// Copyright (c) 2022  Carnegie Mellon University
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

#include "nav2_lifecycle_manager/lifecycle_manager.hpp"
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <diagnostic_updater/publisher.hpp>


namespace cabot_lifecycle_manager
{

class CaBotLifecycleManager : public nav2_lifecycle_manager::LifecycleManager
{
public:
  /**
   * @brief A constructor for nav2_lifecycle_manager::LifecycleManager
   */
  CaBotLifecycleManager()
  {
    diagnostics_updater_.add(
      "ROS2 Lifecycle Manager",
      std::bind(&CaBotLifecycleManager::update_status, this, std::placeholders::_1));
  }
  /**
   * @brief A destructor for nav2_lifecycle_manager::LifecycleManager
   */
  ~CaBotLifecycleManager()
  {
  }

private:
  void update_status(diagnostic_updater::DiagnosticStatusWrapper & stat)
  {
    if (system_active_) {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "working");
    } else {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "not working");
    }

    for (auto & node_name : node_names_) {
      if (bond_map_[node_name]->isBroken()) {
        stat.add(node_name, "not working");
      } else {
        stat.add(node_name, "working");
      }
    }
  }
};
}  // namespace cabot_lifecycle_manager

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<cabot_lifecycle_manager::CaBotLifecycleManager>();
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
