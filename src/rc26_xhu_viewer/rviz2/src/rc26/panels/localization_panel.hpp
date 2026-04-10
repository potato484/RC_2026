#ifndef RC26_XHU_VIEWER__PANELS__LOCALIZATION_PANEL_HPP_
#define RC26_XHU_VIEWER__PANELS__LOCALIZATION_PANEL_HPP_

#include <rviz_common/panel.hpp>

class QLabel;

namespace rc26_xhu_viewer
{

class LocalizationPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit LocalizationPanel(QWidget * parent = nullptr);

  void load(const rviz_common::Config & config) override;
  void save(rviz_common::Config config) const override;

private:
  QLabel * health_label_;
  QLabel * backend_status_label_;
  QLabel * route_observability_label_;
  QLabel * reloc_state_label_;
};

}  // namespace rc26_xhu_viewer

#endif  // RC26_XHU_VIEWER__PANELS__LOCALIZATION_PANEL_HPP_
