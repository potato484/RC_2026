#pragma once

#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>

#include "rc26_interfaces/msg/behavior_tree_localization.hpp"
#include "rc26_interfaces/msg/behavior_tree_localization_entry.hpp"
#include "rc26_interfaces/msg/behavior_tree_localized_node.hpp"

#include <filesystem>
#include <map>
#include <string>
#include <vector>

namespace rc26_decision {

class ChineseLocalizationModule {
public:
  ChineseLocalizationModule(rclcpp::Node *node, std::string config_path);

  bool reloadIfChanged();

  rc26_interfaces::msg::BehaviorTreeLocalizedNode
  getChineseNode(const BT::TreeNode &node) const;

  rc26_interfaces::msg::BehaviorTreeLocalizationEntry
  getChineseBlackboardKey(const std::string &key) const;

  rc26_interfaces::msg::BehaviorTreeLocalization
  buildMessage(const std::string &tree_file,
               const std::vector<const BT::TreeNode *> &nodes,
               const std::vector<std::string> &blackboard_keys) const;

  const std::string &locale() const { return locale_; }
  const std::string &version() const { return version_; }
  const std::string &configPath() const { return config_path_; }

private:
  struct LocalizedValueSpec {
    std::string raw_value;
    std::string display_name;
    std::string explanation;
  };

  struct LocalizedEntrySpec {
    std::string display_name;
    std::string tooltip;
    std::string summary;
    std::string scenario;
    std::string attention;
    std::string markdown;
    std::vector<std::string> related_blackboard_keys;
    std::map<std::string, LocalizedValueSpec> values;
  };

  struct CatalogData {
    std::string locale{"zh-CN"};
    std::string version{"builtin-fallback"};
    std::map<std::string, LocalizedEntrySpec> node_types;
    std::map<std::string, LocalizedEntrySpec> registrations;
    std::map<std::string, LocalizedEntrySpec> custom_types;
    std::map<std::string, LocalizedEntrySpec> subtrees;
    std::map<std::string, LocalizedEntrySpec> instances;
    std::map<std::string, LocalizedEntrySpec> full_paths;
    std::map<std::string, LocalizedEntrySpec> blackboard_keys;
    std::map<std::string, LocalizedEntrySpec> services;
    std::map<std::string, LocalizedEntrySpec> events;
  };

  bool loadFromDisk(CatalogData &out_data,
                    std::filesystem::file_time_type *out_write_time) const;

  static std::map<std::string, LocalizedEntrySpec>
  parseSection(const YAML::Node &root, const char *section_name);

  static LocalizedEntrySpec parseEntrySpec(const YAML::Node &node);
  static LocalizedValueSpec parseValueSpec(const std::string &raw_value,
                                           const YAML::Node &node);
  static std::vector<std::string> readStringList(const YAML::Node &node);
  static LocalizedEntrySpec mergeEntry(const LocalizedEntrySpec &base,
                                       const LocalizedEntrySpec &overlay);
  static std::string buildMarkdown(const LocalizedEntrySpec &entry);
  static std::string normalizeMappedKey(const std::string &value);
  static std::string nodeTypeToken(BT::NodeType type);
  static std::string subtreeIdOf(const BT::TreeNode &node);

  LocalizedEntrySpec fallbackEntry(const std::string &display_name,
                                   const std::string &summary) const;

  LocalizedEntrySpec resolveNodeEntry(const BT::TreeNode &node) const;
  std::vector<std::string>
  collectRelatedBlackboardKeys(const BT::TreeNode &node,
                               const LocalizedEntrySpec &entry) const;

  rc26_interfaces::msg::BehaviorTreeLocalizationEntry
  toMsgEntry(const std::string &kind, const std::string &key,
             const LocalizedEntrySpec &entry) const;

  std::string localizedSubtreeName(const std::string &subtree_id) const;

  rclcpp::Node *node_;
  std::string config_path_;
  std::string locale_{"zh-CN"};
  std::string version_{"builtin-fallback"};
  CatalogData catalog_;
  bool has_write_time_{false};
  std::filesystem::file_time_type last_write_time_{};
};

} // namespace rc26_decision
