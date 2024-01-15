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
  path_ = path;
  breadcrumb_ = breadcrumb;
  nice_name_ = breadcrumb;
  RCLCPP_DEBUG(
    rclcpp::get_logger("GenericAnalyzer"), "GenericAnalyzer, breadcrumb: %s", breadcrumb_.c_str());

  std::map<std::string, rclcpp::Parameter> parameters;
  if (!node->get_parameters(breadcrumb, parameters)) {
    RCLCPP_ERROR(
      rclcpp::get_logger("CabotAnalyzer"),
      "Couldn't retrieve parameters for generic analyzer at prefix '%s'.", breadcrumb.c_str());
    return false;
  }

  double timeout = 5.0;
  int num_items_expected = -1;
  bool discard_stale = false;
  
  for (const auto & param : parameters) {
    string pname = param.first;
    rclcpp::Parameter pvalue = param.second;

    if (pname.compare("path") == 0) {
      RCLCPP_DEBUG(
        rclcpp::get_logger("CabotAnalyzer"), "CabotAnalyzer '%s' found path: %s",
        nice_name_.c_str(), pvalue.value_to_string().c_str());
      nice_name_ = pvalue.as_string();
    } else if (pname.compare("find_and_remove_prefix") == 0) {
      RCLCPP_DEBUG(
        rclcpp::get_logger("CabotAnalyzer"),
        "CabotAnalyzer '%s' found find_and_remove_prefix: %s", nice_name_.c_str(),
        pvalue.value_to_string().c_str());
      vector<string> output = pvalue.as_string_array();
      chaff_ = output;
      startswith_ = output;
    } else if (pname.compare("remove_prefix") == 0) {
      RCLCPP_DEBUG(
        rclcpp::get_logger("CabotAnalyzer"), "CabotAnalyzer '%s' found remove_prefix: %s",
        nice_name_.c_str(), pvalue.value_to_string().c_str());
      chaff_ = pvalue.as_string_array();
    } else if (pname.compare("startswith") == 0) {
      RCLCPP_DEBUG(
        rclcpp::get_logger("CabotAnalyzer"), "CabotAnalyzer '%s' found startswith: %s",
        nice_name_.c_str(), pvalue.value_to_string().c_str());
      startswith_ = pvalue.as_string_array();
    } else if (pname.compare("contains") == 0) {
      RCLCPP_DEBUG(
        rclcpp::get_logger("CabotAnalyzer"), "CabotAnalyzer '%s' found contains: %s",
        nice_name_.c_str(), pvalue.value_to_string().c_str());
      contains_ = pvalue.as_string_array();
    } else if (pname.compare("expected") == 0) {
      RCLCPP_DEBUG(
        rclcpp::get_logger("CabotAnalyzer"), "CabotAnalyzer '%s' found expected: %s",
        nice_name_.c_str(), pvalue.value_to_string().c_str());
      for (auto exp : pvalue.as_string_array()) {
        auto item = std::make_shared<diagnostic_aggregator::StatusItem>(exp);
        this->addItem(exp, item);
      }
    } else if (pname.compare("regex") == 0) {
      RCLCPP_DEBUG(
        rclcpp::get_logger("CabotAnalyzer"), "CabotAnalyzer '%s' found regex: %s",
        nice_name_.c_str(), pvalue.value_to_string().c_str());
      for (auto regex : pvalue.as_string_array()) {
        try {
          std::regex re(regex);
          regex_.push_back(re);
        } catch (std::regex_error & e) {
          RCLCPP_ERROR(
            rclcpp::get_logger("CabotAnalyzer"),
            "Attempted to make regex from %s. Caught exception, ignoring value. Exception: %s",
            regex.c_str(), e.what());
        }
      }
    } else if (pname.compare("timeout") == 0) {
      RCLCPP_DEBUG(
        rclcpp::get_logger("CabotAnalyzer"), "CabotAnalyzer '%s' found timeout: %s",
        nice_name_.c_str(), pvalue.value_to_string().c_str());
      timeout = pvalue.as_double();
    } else if (pname.compare("num_items") == 0) {
      RCLCPP_DEBUG(
        rclcpp::get_logger("CabotAnalyzer"), "CabotAnalyzer '%s' found num_items: %s",
        nice_name_.c_str(), pvalue.value_to_string().c_str());
      num_items_expected = static_cast<int>(pvalue.as_int());
    } else if (pname.compare("discard_stale") == 0) {
      RCLCPP_DEBUG(
        rclcpp::get_logger("CabotAnalyzer"), "CabotAnalyzer '%s' found discard_stale: %s",
        nice_name_.c_str(), pvalue.value_to_string().c_str());
      discard_stale = pvalue.as_bool();
    } else if (pname.compare("treat_as_warning") == 0){
      RCLCPP_DEBUG(
        rclcpp::get_logger("CabotAnalyzer"), "CabotAnalyzer '%s' found treat_as_warning: %s",
        nice_name_.c_str(), pvalue.value_to_string().c_str());
      treat_as_warning_ = true;
    }
  }
  
  if (
    startswith_.size() == 0 && name_.size() == 0 && contains_.size() == 0 &&
    expected_.size() == 0 && regex_.size() == 0)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("CabotAnalyzer"),
      "CabotAnalyzer '%s' was not initialized with any way of checking diagnostics."
      "Name: %s, namespace: %s",
      nice_name_.c_str(), path.c_str(), node->get_namespace());
    return false;
  }

  // convert chaff_ to output name format. Fixes #17
  for (size_t i = 0; i < chaff_.size(); i++) {
    chaff_[i] = diagnostic_aggregator::getOutputName(chaff_[i]);
  }

  string my_path;
  if (path == "/") {
    my_path = nice_name_;
  } else {
    my_path = path + "/" + nice_name_;
  }

  if (my_path.find("/") != 0) {
    my_path = "/" + my_path;
  }

  // base
  num_items_expected_ = num_items_expected;
  timeout_ = timeout;
  path_ = path + "/" + nice_name_;
  discard_stale_ = discard_stale;
  breadcrumb_ = breadcrumb;

  if (discard_stale_ && timeout <= 0) {
    RCLCPP_WARN(
      rclcpp::get_logger("generic_analyzer_base"),
      "Cannot discard stale items if no timeout specified. No items will be discarded");
    discard_stale_ = false;
  }

  has_initialized_ = true;

  RCLCPP_INFO(
    rclcpp::get_logger("GenericAnalyzerBase"),
    "Initialized analyzer '%s' with path '%s' and breadcrumb '%s'.", nice_name_.c_str(),
    path_.c_str(), breadcrumb_.c_str());

  return true;
}

CabotAnalyzer::~CabotAnalyzer() {}

bool CabotAnalyzer::match(const string & name)
{
  RCLCPP_DEBUG(
    rclcpp::get_logger("CabotAnalyzer"), "Analyzer '%s' match %s", nice_name_.c_str(),
    name.c_str());

  std::cmatch what;
  for (unsigned int i = 0; i < regex_.size(); ++i) {
    if (std::regex_match(name.c_str(), what, regex_[i])) {
      RCLCPP_INFO(
        rclcpp::get_logger("CabotAnalyzer"), "Analyzer '%s' matches '%s' with regex.",
        nice_name_.c_str(), name.c_str());
      return true;
    }
  }

  for (unsigned int i = 0; i < expected_.size(); ++i) {
    if (name == expected_[i]) {
      RCLCPP_INFO(
        rclcpp::get_logger("CabotAnalyzer"), "Analyzer '%s' matches '%s'.", nice_name_.c_str(),
        name.c_str());
      return true;
    }
  }

  for (unsigned int i = 0; i < name_.size(); ++i) {
    if (name == name_[i]) {
      RCLCPP_INFO(
        rclcpp::get_logger("CabotAnalyzer"), "Analyzer '%s' matches '%s'.", nice_name_.c_str(),
        name.c_str());
      return true;
    }
  }

  for (unsigned int i = 0; i < startswith_.size(); ++i) {
    if (name.find(startswith_[i]) == 0) {
      RCLCPP_INFO(
        rclcpp::get_logger("CabotAnalyzer"), "Analyzer '%s' matches '%s'.", nice_name_.c_str(),
        name.c_str());
      return true;
    }
  }

  for (unsigned int i = 0; i < contains_.size(); ++i) {
    if (name.find(contains_[i]) != string::npos) {
      RCLCPP_INFO(
        rclcpp::get_logger("CabotAnalyzer"), "Analyzer '%s' matches '%s'.", nice_name_.c_str(),
        name.c_str());
      return true;
    }
  }
  return false;
}

bool CabotAnalyzer::analyze(const std::shared_ptr<diagnostic_aggregator::StatusItem> item) {
  RCLCPP_INFO(
      rclcpp::get_logger("CabotAnalyzer"), "Analyzer '%s' analyze, item %s: %s",
      nice_name_.c_str(), item->getName().c_str(), item->getMessage().c_str());

    if (!has_initialized_ && !has_warned_) {
      has_warned_ = true;
      RCLCPP_WARN(
        rclcpp::get_logger(
          "generic_analyzer_base"),
        R"(GenericAnalyzerBase is asked to analyze diagnostics without being initialized.
        init() must be called in order to correctly use this class.)");
    }

    if (!has_initialized_) {
      return false;
    }

    items_[item->getName()] = item;

    return has_initialized_;
  
  return false;
}

vector<std::shared_ptr<diagnostic_msgs::msg::DiagnosticStatus>> CabotAnalyzer::report()
{
  double timeout_ = 5.0;

  RCLCPP_DEBUG(
      rclcpp::get_logger("GenericAnalyzerBase"), "Analyzer '%s' report()", nice_name_.c_str());

    if (!has_initialized_ && !has_warned_) {
      has_warned_ = true;
      RCLCPP_ERROR(
        rclcpp::get_logger(
          "generic_analyzer_base"),
        R"("GenericAnalyzerBase is asked to report diagnostics without being initialized.
        init() must be called in order to correctly use this class.)");
    }
    if (!has_initialized_) {
      std::vector<std::shared_ptr<diagnostic_msgs::msg::DiagnosticStatus>> vec;
      return vec;
    }

    auto header_status = std::make_shared<diagnostic_msgs::msg::DiagnosticStatus>();
    header_status->name = path_;
    header_status->level = 0;
    header_status->message = "OK";

    std::vector<std::shared_ptr<diagnostic_msgs::msg::DiagnosticStatus>> processed;
    processed.push_back(header_status);

    bool all_stale = true;

    auto it = items_.begin();
    while (it != items_.end()) {
      auto name = it->first;
      auto item = it->second;

      bool stale = false;
      if (timeout_ > 0) {
        stale = (clock_->now() - item->getLastUpdateTime()).seconds() > timeout_;
      }

      // Erase item if its stale and we're discarding items
      if (discard_stale_ && stale) {
        items_.erase(it++);
        continue;
      }

      int8_t level = item->getLevel();
      header_status->level = std::max(static_cast<int8_t>(header_status->level), level);

      diagnostic_msgs::msg::KeyValue kv;
      kv.key = name;
      kv.value = item->getMessage();

      header_status->values.push_back(kv);

      all_stale = all_stale && ((level == diagnostic_msgs::msg::DiagnosticStatus::STALE) || stale);

      processed.push_back(item->toStatusMsg(path_, stale));

      if (stale) {
        header_status->level = diagnostic_msgs::msg::DiagnosticStatus::STALE;
      }

      ++it;
    }

    // Header is not stale unless all subs are
    if (all_stale) {
      header_status->level = diagnostic_msgs::msg::DiagnosticStatus::STALE;
    } else if (header_status->level == diagnostic_msgs::msg::DiagnosticStatus::STALE) {
      header_status->level = 2;
    }

    header_status->message = diagnostic_aggregator::valToMsg(header_status->level);

    // If we expect a given number of items, check that we have this number
    if (num_items_expected_ == 0 && items_.empty()) {
      header_status->level = 0;
      header_status->message = "OK";
    } else if (  // NOLINT
      num_items_expected_ > 0 &&
      static_cast<int8_t>(items_.size()) != num_items_expected_)
    {  // NOLINT
      int8_t lvl = 2;
      header_status->level = std::max(lvl, static_cast<int8_t>(header_status->level));

      std::stringstream expec, item;
      expec << num_items_expected_;
      item << items_.size();

      if (!items_.empty()) {
        header_status->message = "Expected " + expec.str() + ", found " + item.str();
      } else {
        header_status->message = "No items found, expected " + expec.str();
      }
    }


  // Base::report();

  // Check and make sure our expected names haven't been removed ...
  vector<string> expected_names_missing;
  bool has_name = false;

  for (unsigned int i = 0; i < expected_.size(); ++i) {
    has_name = false;
    for (unsigned int j = 0; j < processed.size(); ++j) {
      size_t last_slash = processed[j]->name.rfind("/");
      string nice_name = processed[j]->name.substr(last_slash + 1);
      if (nice_name == expected_[i] || nice_name == diagnostic_aggregator::getOutputName(expected_[i])) {
        has_name = true;
        break;
      }

      // Remove chaff, check names
      for (unsigned int k = 0; k < chaff_.size(); ++k) {
        if (nice_name == diagnostic_aggregator::removeLeadingNameChaff(expected_[i], chaff_[k])) {
          has_name = true;
          break;
        }
      }
    }
    if (!has_name) {
      expected_names_missing.push_back(expected_[i]);
    }
  }

  // Check that all processed items aren't stale
  // bool all_stale = true;
  all_stale = true;
  for (unsigned int j = 0; j < processed.size(); ++j) {
    if (processed[j]->level != diagnostic_msgs::msg::DiagnosticStatus::STALE) {
      all_stale = false;
    }
  }

  // Add missing names to header ...
  for (unsigned int i = 0; i < expected_names_missing.size(); ++i) {
    std::shared_ptr<diagnostic_aggregator::StatusItem> item(new diagnostic_aggregator::StatusItem(expected_names_missing[i]));
    processed.push_back(item->toStatusMsg(path_, true));
  }

  for (unsigned int j = 0; j < processed.size(); ++j) {
    // Remove all leading name chaff
    for (unsigned int i = 0; i < chaff_.size(); ++i) {
      processed[j]->name = diagnostic_aggregator::removeLeadingNameChaff(processed[j]->name, chaff_[i]);
    }

    // If we're missing any items, set the header status to error or stale
    if (expected_names_missing.size() > 0 && processed[j]->name == path_) {
      if (!all_stale) {
        processed[j]->level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
        processed[j]->message = "Error";
      } else {
        processed[j]->level = diagnostic_msgs::msg::DiagnosticStatus::STALE;
        processed[j]->message = "All Stale";
      }

      // Add all missing items to header item
      for (unsigned int k = 0; k < expected_names_missing.size(); ++k) {
        diagnostic_msgs::msg::KeyValue kv;
        kv.key = expected_names_missing[k];
        kv.value = "Missing";
        processed[j]->values.push_back(kv);
      }
    }
  }

  if (downLevel() && header_status->level > diagnostic_msgs::msg::DiagnosticStatus::WARN) {
    header_status->level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
  }
  

  return processed;
}

std::string CabotAnalyzer::getPath() const {
  return path_;
}

std::string CabotAnalyzer::getName() const {
  return nice_name_;
}

bool CabotAnalyzer::downLevel() const {
  return treat_as_warning_;
}
}  // namespace cabot_diagnostics
