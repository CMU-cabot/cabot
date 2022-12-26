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
  CaBotLifecycleManager() {
    diagnostics_updater_.add(
        "ROS2 Lifecycle Manager",
        std::bind(&CaBotLifecycleManager::update_status, this, std::placeholders::_1));
  }
  /**
   * @brief A destructor for nav2_lifecycle_manager::LifecycleManager
   */
  ~CaBotLifecycleManager() {
  }

 private:
  void update_status(diagnostic_updater::DiagnosticStatusWrapper &stat) {
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

}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<cabot_lifecycle_manager::CaBotLifecycleManager>();
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
