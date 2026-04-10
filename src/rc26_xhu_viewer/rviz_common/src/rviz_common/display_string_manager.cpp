#include "rviz_common/display_string_manager.hpp"

#include <cstdlib>
#include <string>

#include <QDir>
#include <QFileInfo>
#include <QHash>
#include <QRegExp>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "yaml-cpp/yaml.h"

namespace rviz_common
{
namespace
{

struct Catalog
{
  QHash<QString, QString> labels;
  QHash<QString, QString> descriptions;
  QHash<QString, QString> dialogs;
  QHash<QString, QString> messages;
  QHash<QString, QString> packages;
  bool loaded{false};
};

Catalog & catalog()
{
  static Catalog value;
  return value;
}

bool containsAsciiLetters(const QString & text)
{
  for (const auto ch : text) {
    const ushort code = ch.unicode();
    if ((code >= 'A' && code <= 'Z') || (code >= 'a' && code <= 'z')) {
      return true;
    }
  }
  return false;
}

void loadSection(const YAML::Node & node, const char * key, QHash<QString, QString> * target)
{
  if (!target) {
    return;
  }

  const YAML::Node section = node[key];
  if (!section || !section.IsMap()) {
    return;
  }

  for (const auto & item : section) {
    if (!item.first.IsScalar() || !item.second.IsScalar()) {
      continue;
    }
    target->insert(
      QString::fromStdString(item.first.as<std::string>()),
      QString::fromStdString(item.second.as<std::string>()));
  }
}

QString loadableSourceCandidate()
{
  const char * env_path = std::getenv("RVIZ_COMMON_TERMINOLOGY_PATH");
  if (env_path && *env_path) {
    const QString candidate = QString::fromLocal8Bit(env_path);
    if (QFileInfo::exists(candidate)) {
      return candidate;
    }
  }

  try {
    const auto share_dir = ament_index_cpp::get_package_share_directory("rviz2");
    const QString installed =
      QString::fromStdString(share_dir) + QStringLiteral("/resources/terminology.yaml");
    if (QFileInfo::exists(installed)) {
      return installed;
    }
  } catch (const std::exception &) {
  }

  const auto source_dir =
    QFileInfo(QString::fromUtf8(__FILE__)).absoluteDir().absoluteFilePath(
    QStringLiteral("../../../rc26_xhu_viewer/resources/terminology.yaml"));
  if (QFileInfo::exists(source_dir)) {
    return source_dir;
  }

  return QString();
}

QString translateByMap(const QHash<QString, QString> & table, const QString & raw)
{
  const auto iter = table.find(raw);
  return iter == table.end() ? QString() : iter.value();
}

QString localizeSingleToken(
  const QHash<QString, QString> & labels,
  const QHash<QString, QString> & packages,
  const QString & raw)
{
  QString localized = translateByMap(labels, raw);
  if (!localized.isEmpty()) {
    return localized;
  }

  localized = translateByMap(packages, raw);
  if (!localized.isEmpty()) {
    return localized;
  }

  const int slash_index = raw.lastIndexOf('/');
  if (slash_index > 0 && !raw.startsWith('/')) {
    const QString tail = raw.mid(slash_index + 1);
    localized = translateByMap(labels, tail);
    if (!localized.isEmpty()) {
      return localized;
    }
  }

  return raw;
}

QString localizeStructuredLabel(
  const QHash<QString, QString> & labels,
  const QHash<QString, QString> & packages,
  const QString & raw)
{
  const QString localized = localizeSingleToken(labels, packages, raw);
  if (localized != raw) {
    return localized;
  }

  QRegExp labeled_value_pattern(QStringLiteral("^(.+): (.+)$"));
  if (labeled_value_pattern.exactMatch(raw)) {
    const QString left = labeled_value_pattern.cap(1);
    const QString right = labeled_value_pattern.cap(2);
    const QString localized_left = localizeStructuredLabel(labels, packages, left);
    const QString localized_right = localizeStructuredLabel(labels, packages, right);
    if (localized_left != left || localized_right != right) {
      return QStringLiteral("%1：%2").arg(localized_left, localized_right);
    }
  }

  QRegExp class_value_pattern(QStringLiteral("^(.+) \\(([^()]+)\\)$"));
  if (class_value_pattern.exactMatch(raw)) {
    const QString left = class_value_pattern.cap(1);
    const QString right = class_value_pattern.cap(2);
    const QString localized_left = localizeSingleToken(labels, packages, left);
    const QString localized_right = localizeSingleToken(labels, packages, right);
    if (localized_left != left || localized_right != right) {
      if (localized_right == right) {
        return localized_left;
      }
      return QStringLiteral("%1（%2）").arg(localized_left, localized_right);
    }
  }

  return raw;
}

QString translateStatusPatterns(const QString & raw)
{
  QRegExp fixed_frame_missing_pattern(
    QStringLiteral("^For frame \\[([^\\]]+)\\]: Fixed Frame \\[([^\\]]+)\\] does not exist$"));
  if (fixed_frame_missing_pattern.exactMatch(raw)) {
    return QStringLiteral("坐标系[%1]对应的固定坐标系[%2]不存在。")
      .arg(fixed_frame_missing_pattern.cap(1))
      .arg(fixed_frame_missing_pattern.cap(2));
  }

  QRegExp frame_missing_pattern(
    QStringLiteral("^For frame \\[([^\\]]+)\\]: Frame \\[([^\\]]+)\\] does not exist$"));
  if (frame_missing_pattern.exactMatch(raw)) {
    return QStringLiteral("坐标系[%1]不存在。").arg(frame_missing_pattern.cap(2));
  }

  QRegExp no_tf_data_pattern(QStringLiteral("^No tf data\\.  Actual error: (.+)$"));
  if (no_tf_data_pattern.exactMatch(raw)) {
    const QString nested = translateStatusPatterns(no_tf_data_pattern.cap(1));
    return QStringLiteral("暂无坐标变换数据。实际错误：%1")
      .arg(nested.isEmpty() ? no_tf_data_pattern.cap(1) : nested);
  }

  QRegExp length_pattern(
    QStringLiteral(
      "^\\[Length: ([^\\]]+)m\\] Click on two points to measure their distance\\. Right-click to reset\\.$"));
  if (length_pattern.exactMatch(raw)) {
    return QStringLiteral("[长度：%1 米] 点击两个点测量距离。右键重置。")
      .arg(length_pattern.cap(1));
  }

  QRegExp point_pattern(
    QStringLiteral("^<b>Left-Click:</b> Select this point\\. \\[(.+)\\]$"));
  if (point_pattern.exactMatch(raw)) {
    return QStringLiteral("<b>左键：</b>选择该点。[%1]").arg(point_pattern.cap(1));
  }

  QRegExp focus_pattern(
    QStringLiteral("^<b>Left-Click:</b> Focus on this point\\. \\[(.+)\\]$"));
  if (focus_pattern.exactMatch(raw)) {
    return QStringLiteral("<b>左键：</b>聚焦到该点。[%1]").arg(focus_pattern.cap(1));
  }

  QRegExp transform_plain_pattern(
    QStringLiteral("^Could not transform from \\[([^\\]]+)\\] to \\[([^\\]]+)\\]$"));
  if (transform_plain_pattern.exactMatch(raw)) {
    return QStringLiteral("无法将[%1]转换到[%2]。")
      .arg(transform_plain_pattern.cap(1))
      .arg(transform_plain_pattern.cap(2));
  }

  QRegExp transform_extra_pattern(
    QStringLiteral("^Could not transform (.+) from \\[([^\\]]+)\\] to \\[([^\\]]+)\\]$"));
  if (transform_extra_pattern.exactMatch(raw)) {
    return QStringLiteral("无法将%1从[%2]转换到[%3]。")
      .arg(transform_extra_pattern.cap(1))
      .arg(transform_extra_pattern.cap(2))
      .arg(transform_extra_pattern.cap(3));
  }

  QRegExp subscribe_error_pattern(QStringLiteral("^Error subscribing: (.+)$"));
  if (subscribe_error_pattern.exactMatch(raw)) {
    return QStringLiteral("订阅失败：%1").arg(subscribe_error_pattern.cap(1));
  }

  QRegExp message_rate_pattern(
    QStringLiteral("^([0-9]+) messages received at ([0-9.]+) hz\\.$"));
  if (message_rate_pattern.exactMatch(raw)) {
    return QStringLiteral("已收到 %1 条消息，当前频率 %2 Hz。")
      .arg(message_rate_pattern.cap(1))
      .arg(message_rate_pattern.cap(2));
  }

  QRegExp message_count_pattern(QStringLiteral("^([0-9]+) messages received$"));
  if (message_count_pattern.exactMatch(raw)) {
    return QStringLiteral("已收到 %1 条消息。").arg(message_count_pattern.cap(1));
  }

  QRegExp depth_map_count_pattern(QStringLiteral("^([0-9]+) depth maps received$"));
  if (depth_map_count_pattern.exactMatch(raw)) {
    return QStringLiteral("已收到 %1 幅深度图。").arg(depth_map_count_pattern.cap(1));
  }

  QRegExp lost_message_pattern(
    QStringLiteral(
      "^Some messages were lost:\\n>\\tNumber of new lost messages: ([^\\n]+) \\n>\\t"
      "Total number of messages lost: ([^\\n]+)$"));
  if (lost_message_pattern.exactMatch(raw)) {
    return QStringLiteral("检测到消息丢失：\n>\t新增丢失消息数：%1\n>\t累计丢失消息数：%2")
      .arg(lost_message_pattern.cap(1))
      .arg(lost_message_pattern.cap(2));
  }

  return QString();
}

QString localizeExactOrEmpty(const QHash<QString, QString> & primary, const QString & raw)
{
  QString localized = translateByMap(primary, raw);
  if (!localized.isEmpty()) {
    return localized;
  }
  localized = translateStatusPatterns(raw);
  return localized;
}

}  // namespace

DisplayStringManager::DisplayStringManager() = default;

const DisplayStringManager & DisplayStringManager::instance()
{
  static DisplayStringManager manager;
  manager.ensureLoaded();
  return manager;
}

void DisplayStringManager::ensureLoaded() const
{
  Catalog & value = catalog();
  if (value.loaded) {
    return;
  }

  value.loaded = true;
  const QString path = loadableSourceCandidate();
  if (path.isEmpty()) {
    return;
  }

  try {
    const auto root = YAML::LoadFile(path.toStdString());
    loadSection(root, "labels", &value.labels);
    loadSection(root, "descriptions", &value.descriptions);
    loadSection(root, "dialogs", &value.dialogs);
    loadSection(root, "messages", &value.messages);
    loadSection(root, "packages", &value.packages);
  } catch (const std::exception &) {
  }
}

QString DisplayStringManager::localizeLabel(const QString & raw) const
{
  ensureLoaded();
  Catalog & value = catalog();
  return localizeStructuredLabel(value.labels, value.packages, raw);
}

QString DisplayStringManager::localizeValue(const QString & raw) const
{
  ensureLoaded();
  Catalog & value = catalog();
  QString localized = localizeStructuredLabel(value.labels, value.packages, raw);
  if (localized != raw) {
    return localized;
  }
  localized = translateByMap(value.messages, raw);
  if (!localized.isEmpty()) {
    return localized;
  }
  localized = translateStatusPatterns(raw);
  return localized.isEmpty() ? raw : localized;
}

QString DisplayStringManager::localizeDescription(const QString & raw) const
{
  ensureLoaded();
  Catalog & value = catalog();
  QString localized = localizeExactOrEmpty(value.descriptions, raw);
  if (!localized.isEmpty()) {
    return localized;
  }
  localized = translateByMap(value.messages, raw);
  if (!localized.isEmpty()) {
    return localized;
  }
  if (containsAsciiLetters(raw)) {
    return QStringLiteral("当前属性说明暂未补充中文。");
  }
  return raw;
}

QString DisplayStringManager::localizeDialogText(const QString & raw) const
{
  ensureLoaded();
  Catalog & value = catalog();
  QString localized = translateByMap(value.dialogs, raw);
  if (!localized.isEmpty()) {
    return localized;
  }
  return localizeStructuredLabel(value.labels, value.packages, raw);
}

QString DisplayStringManager::localizeStatusText(const QString & raw) const
{
  ensureLoaded();
  Catalog & value = catalog();
  QString localized = localizeExactOrEmpty(value.messages, raw);
  if (!localized.isEmpty()) {
    return localized;
  }
  localized = localizeStructuredLabel(value.labels, value.packages, raw);
  return localized;
}

QString DisplayStringManager::localizePackage(const QString & raw) const
{
  ensureLoaded();
  Catalog & value = catalog();
  return localizeStructuredLabel(value.labels, value.packages, raw);
}

QString DisplayStringManager::localizePluginDescription(const QString & raw) const
{
  ensureLoaded();
  Catalog & value = catalog();
  QString localized = localizeExactOrEmpty(value.descriptions, raw);
  if (!localized.isEmpty()) {
    return localized;
  }
  if (containsAsciiLetters(raw)) {
    return QStringLiteral("当前项暂未补充中文说明。");
  }
  return raw;
}

QString DisplayStringManager::resolveRawFromDisplay(
  const QString & display_text,
  const QStringList & raw_candidates) const
{
  ensureLoaded();
  for (const auto & raw : raw_candidates) {
    if (display_text == raw || display_text == localizeLabel(raw)) {
      return raw;
    }
  }
  return display_text;
}

QString DisplayStringManager::rawTooltip(const QString & raw, const QString & localized) const
{
  return (!raw.isEmpty() && raw != localized) ? raw : QString();
}

}  // namespace rviz_common
