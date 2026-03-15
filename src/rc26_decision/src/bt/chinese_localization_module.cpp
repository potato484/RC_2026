#include "rc26_decision/bt/chinese_localization_module.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cctype>
#include <regex>
#include <set>
#include <sstream>
#include <utility>

namespace rc26_decision {

namespace {

std::string readString(const YAML::Node &node, const char *key) {
  const auto value = node[key];
  if (!value || !value.IsScalar()) {
    return {};
  }
  return value.as<std::string>();
}

void appendUnique(std::vector<std::string> &dst, const std::vector<std::string> &src) {
  for (const auto &item : src) {
    if (item.empty()) {
      continue;
    }
    if (std::find(dst.begin(), dst.end(), item) == dst.end()) {
      dst.push_back(item);
    }
  }
}

} // namespace

ChineseLocalizationModule::ChineseLocalizationModule(rclcpp::Node *node,
                                                     std::string config_path)
    : node_(node), config_path_(std::move(config_path)) {
  CatalogData loaded;
  std::filesystem::file_time_type write_time{};
  if (loadFromDisk(loaded, &write_time)) {
    catalog_ = std::move(loaded);
    locale_ = catalog_.locale;
    version_ = catalog_.version;
    last_write_time_ = write_time;
    has_write_time_ = true;
    RCLCPP_INFO(node_->get_logger(), "BT 本地化配置已加载: %s",
                config_path_.c_str());
    return;
  }

  RCLCPP_WARN(node_->get_logger(),
              "BT 本地化配置加载失败，回退到英文原名显示: %s",
              config_path_.c_str());
}

bool ChineseLocalizationModule::reloadIfChanged() {
  std::error_code ec;
  if (!std::filesystem::exists(config_path_, ec) || ec) {
    return false;
  }

  auto current_write_time = std::filesystem::last_write_time(config_path_, ec);
  if (ec) {
    return false;
  }

  if (has_write_time_ && current_write_time == last_write_time_) {
    return false;
  }

  CatalogData loaded;
  if (!loadFromDisk(loaded, &current_write_time)) {
    RCLCPP_WARN(node_->get_logger(),
                "BT 本地化配置热更新失败，继续使用旧配置: %s",
                config_path_.c_str());
    return false;
  }

  catalog_ = std::move(loaded);
  locale_ = catalog_.locale;
  version_ = catalog_.version;
  last_write_time_ = current_write_time;
  has_write_time_ = true;
  RCLCPP_INFO(node_->get_logger(), "BT 本地化配置已热更新: %s",
              config_path_.c_str());
  return true;
}

rc26_interfaces::msg::BehaviorTreeLocalizedNode
ChineseLocalizationModule::getChineseNode(const BT::TreeNode &node) const {
  const auto resolved = resolveNodeEntry(node);
  const auto subtree_id = subtreeIdOf(node);

  rc26_interfaces::msg::BehaviorTreeLocalizedNode msg;
  msg.uid = node.UID();
  msg.locale = locale_;
  msg.display_name =
      resolved.display_name.empty() ? node.name() : resolved.display_name;
  msg.original_name = node.name();
  msg.registration_name = node.registrationName();
  msg.registration_display_name = node.registrationName();
  const auto reg_it = catalog_.registrations.find(node.registrationName());
  if (reg_it != catalog_.registrations.end() &&
      !reg_it->second.display_name.empty()) {
    msg.registration_display_name = reg_it->second.display_name;
  }
  const auto custom_it = catalog_.custom_types.find(node.registrationName());
  if (custom_it != catalog_.custom_types.end() &&
      !custom_it->second.display_name.empty()) {
    msg.registration_display_name = custom_it->second.display_name;
  }
  msg.subtree_id = subtree_id;
  msg.subtree_display_name = localizedSubtreeName(subtree_id);
  msg.tooltip = resolved.tooltip;
  msg.summary = resolved.summary;
  msg.scenario = resolved.scenario;
  msg.attention = resolved.attention;
  msg.markdown = resolved.markdown.empty() ? buildMarkdown(resolved)
                                           : resolved.markdown;
  msg.related_blackboard_keys = collectRelatedBlackboardKeys(node, resolved);
  return msg;
}

rc26_interfaces::msg::BehaviorTreeLocalizationEntry
ChineseLocalizationModule::getChineseBlackboardKey(const std::string &key) const {
  const auto it = catalog_.blackboard_keys.find(key);
  if (it != catalog_.blackboard_keys.end()) {
    return toMsgEntry("blackboard_key", key, it->second);
  }

  const auto summary =
      "当前还没有单独维护中文解释，先按原始黑板键展示。";
  return toMsgEntry("blackboard_key", key, fallbackEntry(key, summary));
}

rc26_interfaces::msg::BehaviorTreeLocalization
ChineseLocalizationModule::buildMessage(
    const std::string &tree_file, const std::vector<const BT::TreeNode *> &nodes,
    const std::vector<std::string> &blackboard_keys) const {
  rc26_interfaces::msg::BehaviorTreeLocalization msg;
  msg.tree_file = tree_file;
  msg.locale = locale_;
  msg.version = version_;

  for (const auto *node : nodes) {
    if (!node) {
      continue;
    }
    msg.nodes.push_back(getChineseNode(*node));
  }

  std::set<std::string> merged_blackboard_keys(blackboard_keys.begin(),
                                               blackboard_keys.end());
  for (const auto &[key, _] : catalog_.blackboard_keys) {
    merged_blackboard_keys.insert(key);
  }
  for (const auto &key : merged_blackboard_keys) {
    msg.blackboard_keys.push_back(getChineseBlackboardKey(key));
  }

  const auto append_section =
      [this, &msg](const std::string &kind,
                   const std::map<std::string, LocalizedEntrySpec> &section) {
        for (const auto &[key, entry] : section) {
          msg.catalog_entries.push_back(toMsgEntry(kind, key, entry));
        }
      };

  append_section("node_type", catalog_.node_types);
  append_section("registration", catalog_.registrations);
  append_section("custom_type", catalog_.custom_types);
  append_section("subtree", catalog_.subtrees);
  append_section("instance", catalog_.instances);
  append_section("full_path", catalog_.full_paths);
  append_section("service", catalog_.services);
  append_section("event", catalog_.events);
  return msg;
}

bool ChineseLocalizationModule::loadFromDisk(
    CatalogData &out_data,
    std::filesystem::file_time_type *out_write_time) const {
  std::error_code ec;
  if (!std::filesystem::exists(config_path_, ec) || ec) {
    return false;
  }

  YAML::Node root;
  try {
    root = YAML::LoadFile(config_path_);
  } catch (const std::exception &e) {
    RCLCPP_WARN(node_->get_logger(), "BT 本地化配置解析失败: %s",
                e.what());
    return false;
  }

  out_data.locale = readString(root, "locale");
  if (out_data.locale.empty()) {
    out_data.locale = "zh-CN";
  }
  out_data.version = readString(root, "version");
  if (out_data.version.empty()) {
    out_data.version = "file";
  }
  out_data.node_types = parseSection(root, "node_types");
  out_data.registrations = parseSection(root, "registrations");
  out_data.custom_types = parseSection(root, "custom_types");
  out_data.subtrees = parseSection(root, "subtrees");
  out_data.instances = parseSection(root, "instances");
  out_data.full_paths = parseSection(root, "full_paths");
  out_data.blackboard_keys = parseSection(root, "blackboard_keys");
  out_data.services = parseSection(root, "services");
  out_data.events = parseSection(root, "events");

  if (out_write_time != nullptr) {
    *out_write_time = std::filesystem::last_write_time(config_path_, ec);
  }
  return true;
}

std::map<std::string, ChineseLocalizationModule::LocalizedEntrySpec>
ChineseLocalizationModule::parseSection(const YAML::Node &root,
                                        const char *section_name) {
  std::map<std::string, LocalizedEntrySpec> result;
  const auto section = root[section_name];
  if (!section || !section.IsMap()) {
    return result;
  }

  for (const auto &item : section) {
    if (!item.first.IsScalar() || !item.second.IsMap()) {
      continue;
    }
    result.emplace(item.first.as<std::string>(), parseEntrySpec(item.second));
  }
  return result;
}

ChineseLocalizationModule::LocalizedEntrySpec
ChineseLocalizationModule::parseEntrySpec(const YAML::Node &node) {
  LocalizedEntrySpec entry;
  entry.display_name = readString(node, "display_name");
  entry.tooltip = readString(node, "tooltip");
  entry.summary = readString(node, "summary");
  entry.scenario = readString(node, "scenario");
  entry.attention = readString(node, "attention");
  entry.markdown = readString(node, "markdown");
  entry.related_blackboard_keys = readStringList(node["related_blackboard_keys"]);

  const auto values = node["values"];
  if (values && values.IsMap()) {
    for (const auto &item : values) {
      if (!item.first.IsScalar()) {
        continue;
      }
      const auto raw_value = item.first.as<std::string>();
      entry.values.emplace(raw_value, parseValueSpec(raw_value, item.second));
    }
  }

  if (entry.tooltip.empty()) {
    entry.tooltip = entry.summary;
  }
  if (entry.markdown.empty()) {
    entry.markdown = buildMarkdown(entry);
  }
  return entry;
}

ChineseLocalizationModule::LocalizedValueSpec
ChineseLocalizationModule::parseValueSpec(const std::string &raw_value,
                                          const YAML::Node &node) {
  LocalizedValueSpec value;
  value.raw_value = raw_value;
  value.display_name = readString(node, "display_name");
  value.explanation = readString(node, "explanation");
  if (value.display_name.empty()) {
    value.display_name = raw_value;
  }
  return value;
}

std::vector<std::string>
ChineseLocalizationModule::readStringList(const YAML::Node &node) {
  std::vector<std::string> values;
  if (!node || !node.IsSequence()) {
    return values;
  }
  for (const auto &item : node) {
    if (item.IsScalar()) {
      values.push_back(item.as<std::string>());
    }
  }
  return values;
}

ChineseLocalizationModule::LocalizedEntrySpec
ChineseLocalizationModule::mergeEntry(const LocalizedEntrySpec &base,
                                      const LocalizedEntrySpec &overlay) {
  LocalizedEntrySpec merged = base;
  if (!overlay.display_name.empty()) {
    merged.display_name = overlay.display_name;
  }
  if (!overlay.tooltip.empty()) {
    merged.tooltip = overlay.tooltip;
  }
  if (!overlay.summary.empty()) {
    merged.summary = overlay.summary;
  }
  if (!overlay.scenario.empty()) {
    merged.scenario = overlay.scenario;
  }
  if (!overlay.attention.empty()) {
    merged.attention = overlay.attention;
  }
  if (!overlay.markdown.empty()) {
    merged.markdown = overlay.markdown;
  }
  appendUnique(merged.related_blackboard_keys, overlay.related_blackboard_keys);
  for (const auto &[raw_value, spec] : overlay.values) {
    merged.values[raw_value] = spec;
  }
  if (merged.tooltip.empty()) {
    merged.tooltip = merged.summary;
  }
  if (merged.markdown.empty()) {
    merged.markdown = buildMarkdown(merged);
  }
  return merged;
}

std::string
ChineseLocalizationModule::buildMarkdown(const LocalizedEntrySpec &entry) {
  std::ostringstream oss;
  if (!entry.summary.empty()) {
    oss << "## 功能说明\n" << entry.summary << "\n";
  }
  if (!entry.scenario.empty()) {
    if (oss.tellp() > 0) {
      oss << "\n";
    }
    oss << "## 使用场景\n" << entry.scenario << "\n";
  }
  if (!entry.attention.empty()) {
    if (oss.tellp() > 0) {
      oss << "\n";
    }
    oss << "## 注意事项\n" << entry.attention << "\n";
  }
  if (!entry.related_blackboard_keys.empty()) {
    if (oss.tellp() > 0) {
      oss << "\n";
    }
    oss << "## 相关黑板键\n";
    for (const auto &key : entry.related_blackboard_keys) {
      oss << "- " << key << "\n";
    }
  }
  return oss.str();
}

std::string
ChineseLocalizationModule::normalizeMappedKey(const std::string &value) {
  const auto begin = value.find_first_not_of(" \t\r\n");
  if (begin == std::string::npos) {
    return {};
  }
  const auto end = value.find_last_not_of(" \t\r\n");
  const auto trimmed = value.substr(begin, end - begin + 1U);
  static const std::regex kIdentifier(R"(^[A-Za-z_][A-Za-z0-9_]*$)");
  std::smatch match;
  if (std::regex_match(trimmed, match, std::regex(R"(^\{(.+)\}$)"))) {
    return match[1].str();
  }
  if (std::regex_match(trimmed, kIdentifier)) {
    return trimmed;
  }
  return {};
}

std::string ChineseLocalizationModule::nodeTypeToken(BT::NodeType type) {
  switch (type) {
  case BT::NodeType::ACTION:
    return "ACTION";
  case BT::NodeType::CONDITION:
    return "CONDITION";
  case BT::NodeType::CONTROL:
    return "CONTROL";
  case BT::NodeType::DECORATOR:
    return "DECORATOR";
  case BT::NodeType::SUBTREE:
    return "SUBTREE";
  default:
    return "UNDEFINED";
  }
}

std::string ChineseLocalizationModule::subtreeIdOf(const BT::TreeNode &node) {
  const auto path = node.fullPath();
  const auto slash_pos = path.find('/');
  return slash_pos == std::string::npos ? path : path.substr(0, slash_pos);
}

ChineseLocalizationModule::LocalizedEntrySpec
ChineseLocalizationModule::fallbackEntry(const std::string &display_name,
                                         const std::string &summary) const {
  LocalizedEntrySpec entry;
  entry.display_name = display_name;
  entry.tooltip = summary;
  entry.summary = summary;
  entry.scenario = "当前版本尚未单独维护示例场景，可先结合时间线和黑板值判断。";
  entry.attention = "如果该项长期需要中文解释，建议补到 bt_localization.yaml。";
  entry.markdown = buildMarkdown(entry);
  return entry;
}

ChineseLocalizationModule::LocalizedEntrySpec
ChineseLocalizationModule::resolveNodeEntry(const BT::TreeNode &node) const {
  auto resolved = fallbackEntry(node.name(), "当前版本还没有为这个节点补充中文解释。");

  const auto type_it = catalog_.node_types.find(nodeTypeToken(node.type()));
  if (type_it != catalog_.node_types.end()) {
    auto generic = type_it->second;
    generic.display_name.clear();
    resolved = mergeEntry(resolved, generic);
  }
  const auto reg_it = catalog_.registrations.find(node.registrationName());
  if (reg_it != catalog_.registrations.end()) {
    resolved = mergeEntry(resolved, reg_it->second);
  }
  const auto custom_it = catalog_.custom_types.find(node.registrationName());
  if (custom_it != catalog_.custom_types.end()) {
    resolved = mergeEntry(resolved, custom_it->second);
  }
  if (node.type() == BT::NodeType::SUBTREE) {
    const auto subtree_target_it = catalog_.subtrees.find(node.name());
    if (subtree_target_it != catalog_.subtrees.end()) {
      resolved = mergeEntry(resolved, subtree_target_it->second);
    }
  }
  const auto instance_it = catalog_.instances.find(node.name());
  if (instance_it != catalog_.instances.end()) {
    resolved = mergeEntry(resolved, instance_it->second);
  }
  const auto path_it = catalog_.full_paths.find(node.fullPath());
  if (path_it != catalog_.full_paths.end()) {
    resolved = mergeEntry(resolved, path_it->second);
  }

  if (resolved.display_name.empty()) {
    resolved.display_name = node.name();
  }
  if (resolved.tooltip.empty()) {
    resolved.tooltip = resolved.summary;
  }
  if (resolved.markdown.empty()) {
    resolved.markdown = buildMarkdown(resolved);
  }
  return resolved;
}

std::vector<std::string> ChineseLocalizationModule::collectRelatedBlackboardKeys(
    const BT::TreeNode &node, const LocalizedEntrySpec &entry) const {
  std::vector<std::string> keys;
  appendUnique(keys, entry.related_blackboard_keys);

  const auto &cfg = node.config();
  for (const auto &[_, remapped] : cfg.input_ports) {
    const auto key = normalizeMappedKey(remapped);
    if (!key.empty()) {
      appendUnique(keys, std::vector<std::string>{key});
    }
  }
  for (const auto &[_, remapped] : cfg.output_ports) {
    const auto key = normalizeMappedKey(remapped);
    if (!key.empty()) {
      appendUnique(keys, std::vector<std::string>{key});
    }
  }
  return keys;
}

rc26_interfaces::msg::BehaviorTreeLocalizationEntry
ChineseLocalizationModule::toMsgEntry(const std::string &kind,
                                      const std::string &key,
                                      const LocalizedEntrySpec &entry) const {
  rc26_interfaces::msg::BehaviorTreeLocalizationEntry msg;
  msg.kind = kind;
  msg.key = key;
  msg.locale = locale_;
  msg.display_name = entry.display_name.empty() ? key : entry.display_name;
  msg.tooltip = entry.tooltip;
  msg.summary = entry.summary;
  msg.scenario = entry.scenario;
  msg.attention = entry.attention;
  msg.markdown = entry.markdown.empty() ? buildMarkdown(entry) : entry.markdown;
  msg.related_blackboard_keys = entry.related_blackboard_keys;
  for (const auto &[raw_value, spec] : entry.values) {
    rc26_interfaces::msg::BehaviorTreeLocalizedValue value_msg;
    value_msg.raw_value = raw_value;
    value_msg.display_name =
        spec.display_name.empty() ? raw_value : spec.display_name;
    value_msg.explanation = spec.explanation;
    msg.values.push_back(std::move(value_msg));
  }
  return msg;
}

std::string
ChineseLocalizationModule::localizedSubtreeName(const std::string &subtree_id) const {
  const auto it = catalog_.subtrees.find(subtree_id);
  if (it == catalog_.subtrees.end() || it->second.display_name.empty()) {
    return subtree_id;
  }
  return it->second.display_name;
}

} // namespace rc26_decision
