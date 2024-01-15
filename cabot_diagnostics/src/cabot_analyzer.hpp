#ifndef DIAGNOSTIC_AGGREGATOR__CABOT_ANALYZER_HPP_
#define DIAGNOSTIC_AGGREGATOR__CABOT_ANALYZER_HPP_

#include <map>
#include <memory>
#include <regex>
#include <sstream>
#include <string>
#include <vector>

#include "diagnostic_aggregator/analyzer.hpp"
#include "diagnostic_aggregator/generic_analyzer_base.hpp"
#include "diagnostic_aggregator/status_item.hpp"
#include "diagnostic_aggregator/visibility_control.hpp"

#include "diagnostic_msgs/msg/diagnostic_status.h"
#include "diagnostic_msgs/msg/key_value.h"

#include "pluginlib/class_list_macros.hpp"

#include "rclcpp/logger.hpp"
#include "rclcpp/rclcpp.hpp"

namespace cabot_diagnostics
{
class CabotAnalyzer : public diagnostic_aggregator::Analyzer
{
public:
  /*!
   *\brief Default constructor loaded by pluginlib
   */
  DIAGNOSTIC_AGGREGATOR_PUBLIC
  CabotAnalyzer();

  DIAGNOSTIC_AGGREGATOR_PUBLIC
  virtual ~CabotAnalyzer();

  DIAGNOSTIC_AGGREGATOR_PUBLIC
  bool init(
      const std::string & base_path, const std::string & breadcrumb,
      const rclcpp::Node::SharedPtr node);
  
  DIAGNOSTIC_AGGREGATOR_PUBLIC
  bool match(const std::string & name);
  
  DIAGNOSTIC_AGGREGATOR_PUBLIC
  bool analyze(const std::shared_ptr<diagnostic_aggregator::StatusItem> item);

  DIAGNOSTIC_AGGREGATOR_PUBLIC
  std::vector<std::shared_ptr<diagnostic_msgs::msg::DiagnosticStatus>> report();

  DIAGNOSTIC_AGGREGATOR_PUBLIC
  std::string getPath() const;

  DIAGNOSTIC_AGGREGATOR_PUBLIC
  std::string getName() const;

  DIAGNOSTIC_AGGREGATOR_PUBLIC
  bool downLevel() const;

protected:
  double timeout_;
  int num_items_expected_;
  std::string breadcrumb_;

  void addItem(std::string name, std::shared_ptr<diagnostic_aggregator::StatusItem> item)
  {
    items_[name] = item;
  }

private:
  std::string path_;
  std::string nice_name_;
  std::vector<std::string> chaff_; /**< Removed from the start of node names. */
  std::vector<std::string> expected_;
  std::vector<std::string> startswith_;
  std::vector<std::string> contains_;
  std::vector<std::string> name_;
  std::vector<std::regex> regex_; /**< Regular expressions to check against diagnostics names. */

  std::map<std::string, std::shared_ptr<diagnostic_aggregator::StatusItem>> items_;
  bool discard_stale_, has_initialized_, has_warned_, treat_as_warning_;
};

}  // namespace cabot_diagnostics

#endif  // DIAGNOSTIC_AGGREGATOR__GENERIC_ANALYZER_HPP_
