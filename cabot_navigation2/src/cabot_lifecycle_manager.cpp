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
    updater_ = std::make_shared<diagnostic_updater::Updater>(this);
    updater_->setHardwareID("cabot lifecycle manager");
    updater_->add("Lifecycle Manager", std::bind(&CaBotLifecycleManager::update_status, this, std::placeholders::_1));
  }
  /**
   * @brief A destructor for nav2_lifecycle_manager::LifecycleManager
   */
  ~CaBotLifecycleManager() {
  }

  std::shared_ptr<diagnostic_updater::Updater> updater_;

 private:
  void update_status(diagnostic_updater::DiagnosticStatusWrapper &stat) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "working");
  }

};

}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<cabot_lifecycle_manager::CaBotLifecycleManager>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();

  return 0;
}
