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

private:
  diagnostic_aggregator::GenericAnalyzer analyzer_;
  bool treat_as_warning_;
};

}  // namespace cabot_diagnostics

#endif  // DIAGNOSTIC_AGGREGATOR__GENERIC_ANALYZER_HPP_
