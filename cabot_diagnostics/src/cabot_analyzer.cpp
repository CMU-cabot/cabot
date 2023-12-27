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
  path_ = path;

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

    if (pname.compare("path") == 0) {
      RCLCPP_DEBUG(
        rclcpp::get_logger("GenericAnalyzer"), "GenericAnalyzer '%s' found path: %s",
        nice_name_.c_str(), pvalue.value_to_string().c_str());
      nice_name_ = pvalue.as_string();
    }
  }
  return true;
}

CabotAnalyzer::~CabotAnalyzer() {}

bool CabotAnalyzer::match(const string & name)
{
  RCLCPP_INFO(rclcpp::get_logger("CabotAnalyzer"),
              "match with %s", name.c_str());
  return false;
}

bool CabotAnalyzer::analyze(const std::shared_ptr<diagnostic_aggregator::StatusItem> item) {
  RCLCPP_INFO(rclcpp::get_logger("CabotAnalyzer"),
              "analyze item %s", item->getName().c_str());
  return false;
}

vector<std::shared_ptr<diagnostic_msgs::msg::DiagnosticStatus>> CabotAnalyzer::report()
{
  vector<std::shared_ptr<diagnostic_msgs::msg::DiagnosticStatus>> processed;

  diagnostic_msgs::msg::DiagnosticStatus::SharedPtr status
      = std::make_shared<diagnostic_msgs::msg::DiagnosticStatus>();
  status->name = getPath()+"/"+getName();

  // set level and message
  status->level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  status->message = "set your message";

  processed.push_back(status);

  return processed;
}

std::string CabotAnalyzer::getPath() const {
  return path_;
}

std::string CabotAnalyzer::getName() const {
  return nice_name_;
}
}  // namespace cabot_diagnostics
