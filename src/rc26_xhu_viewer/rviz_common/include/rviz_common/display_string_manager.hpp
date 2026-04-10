#ifndef RVIZ_COMMON__DISPLAY_STRING_MANAGER_HPP_
#define RVIZ_COMMON__DISPLAY_STRING_MANAGER_HPP_

#include <QString>
#include <QStringList>

#include "rviz_common/visibility_control.hpp"

namespace rviz_common
{

class RVIZ_COMMON_PUBLIC DisplayStringManager
{
public:
  static const DisplayStringManager & instance();

  QString localizeLabel(const QString & raw) const;
  QString localizeValue(const QString & raw) const;
  QString localizeDescription(const QString & raw) const;
  QString localizeDialogText(const QString & raw) const;
  QString localizeStatusText(const QString & raw) const;
  QString localizePackage(const QString & raw) const;
  QString localizePluginDescription(const QString & raw) const;

  QString resolveRawFromDisplay(
    const QString & display_text,
    const QStringList & raw_candidates) const;

  QString rawTooltip(const QString & raw, const QString & localized) const;

private:
  DisplayStringManager();
  void ensureLoaded() const;

  DisplayStringManager(const DisplayStringManager &) = delete;
  DisplayStringManager & operator=(const DisplayStringManager &) = delete;
};

}  // namespace rviz_common

#endif  // RVIZ_COMMON__DISPLAY_STRING_MANAGER_HPP_
