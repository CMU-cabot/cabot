#include "diagnostic_aggregator/generic_analyzer.hpp"
#include "cabot_analyzer.hpp"

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/parameter.hpp"

PLUGINLIB_EXPORT_CLASS(cabot_diagnostics::CabotAnalyzer, diagnostic_aggregator::Analyzer)

namespace cabot_diagnostics
{
using std::string;
using std::vector;

CabotAnalyzer::CabotAnalyzer() {}

bool CabotAnalyzer::init(
  const std::string & path, const std::string & breadcrumb, const rclcpp::Node::SharedPtr node)
{
  treat_as_warning_ = false;
  RCLCPP_DEBUG(
    rclcpp::get_logger("CabotAnalyzer"), "CabotAnalyzer, breadcrumb: %s", breadcrumb.c_str());

  std::map<std::string, rclcpp::Parameter> parameters;
  if (!node->get_parameters(breadcrumb, parameters)) {
    RCLCPP_ERROR(
      rclcpp::get_logger("CabotAnalyzer"),
      "Couldn't retrieve parameters for generic analyzer at prefix '%s'.", breadcrumb.c_str());
    return false;
  }
  
  for (const auto & param : parameters) {
    string pname = param.first;
    rclcpp::Parameter pvalue = param.second;
    if (pname.compare("treat_as_warning") == 0){
      RCLCPP_DEBUG(
        rclcpp::get_logger("CabotAnalyzer"), "CabotAnalyzer '%s' found treat_as_warning: %s",
        getName().c_str(), pvalue.value_to_string().c_str());
      treat_as_warning_ = true;
    }
  }

  return analyzer.init(path, breadcrumb, node);
}

CabotAnalyzer::~CabotAnalyzer() {}

bool CabotAnalyzer::match(const string & name)
{
  RCLCPP_DEBUG(
    rclcpp::get_logger("CabotAnalyzer"), "CabotAnalyzer '%s' match %s", getName().c_str(),
    name.c_str());

  return analyzer.match(name);
}

bool CabotAnalyzer::analyze(const std::shared_ptr<diagnostic_aggregator::StatusItem> item) {
  RCLCPP_DEBUG(
      rclcpp::get_logger("CabotAnalyzer"), "CabotAnalyzer '%s' analyze, item %s: %s",
      getName().c_str(), item->getName().c_str(), item->getMessage().c_str());
  
  return analyzer.analyze(item);
}

vector<std::shared_ptr<diagnostic_msgs::msg::DiagnosticStatus>> CabotAnalyzer::report()
{
  RCLCPP_DEBUG(
      rclcpp::get_logger("CabotAnalyzer"), "CabotAnalyzer '%s' report()", getName().c_str());
  
  std::vector<std::shared_ptr<diagnostic_msgs::msg::DiagnosticStatus>> processed = analyzer.report();

  if (treat_as_warning_ && processed[0]->level > diagnostic_msgs::msg::DiagnosticStatus::WARN) {
    processed[0]->level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
  }

  return processed;
}

std::string CabotAnalyzer::getPath() const {
  return analyzer.getPath();
}

std::string CabotAnalyzer::getName() const {
  return analyzer.getName();
}

}  // namespace cabot_diagnostics
