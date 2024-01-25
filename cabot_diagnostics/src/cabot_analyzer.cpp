/*******************************************************************************
 * Copyright (c) 2023  Miraikan and Carnegie Mellon University
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *******************************************************************************/

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

  return analyzer_.init(path, breadcrumb, node);
}

CabotAnalyzer::~CabotAnalyzer() {}

bool CabotAnalyzer::match(const string & name)
{
  RCLCPP_DEBUG(
    rclcpp::get_logger("CabotAnalyzer"), "CabotAnalyzer '%s' match %s", getName().c_str(),
    name.c_str());

  return analyzer_.match(name);
}

bool CabotAnalyzer::analyze(const std::shared_ptr<diagnostic_aggregator::StatusItem> item) {
  RCLCPP_DEBUG(
      rclcpp::get_logger("CabotAnalyzer"), "CabotAnalyzer '%s' analyze, item %s: %s",
      getName().c_str(), item->getName().c_str(), item->getMessage().c_str());
  
  return analyzer_.analyze(item);
}

vector<std::shared_ptr<diagnostic_msgs::msg::DiagnosticStatus>> CabotAnalyzer::report()
{
  RCLCPP_DEBUG(
      rclcpp::get_logger("CabotAnalyzer"), "CabotAnalyzer '%s' report()", getName().c_str());
  
  std::vector<std::shared_ptr<diagnostic_msgs::msg::DiagnosticStatus>> processed = analyzer_.report();

  if (treat_as_warning_ && processed[0]->level > diagnostic_msgs::msg::DiagnosticStatus::WARN) {
    processed[0]->level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
  }

  return processed;
}

std::string CabotAnalyzer::getPath() const {
  return analyzer_.getPath();
}

std::string CabotAnalyzer::getName() const {
  return analyzer_.getName();
}

}  // namespace cabot_diagnostics
