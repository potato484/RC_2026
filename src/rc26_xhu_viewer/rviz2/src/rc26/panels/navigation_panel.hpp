#ifndef RC26_XHU_VIEWER__PANELS__NAVIGATION_PANEL_HPP_
#define RC26_XHU_VIEWER__PANELS__NAVIGATION_PANEL_HPP_

#include <rviz_common/panel.hpp>

class QLabel;

namespace rc26_xhu_viewer
{

class NavigationPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit NavigationPanel(QWidget * parent = nullptr);

  void load(const rviz_common::Config & config) override;
  void save(rviz_common::Config config) const override;

private:
  QLabel * motion_mode_label_;
  QLabel * tracking_state_label_;
  QLabel * local_planner_label_;
  QLabel * recovery_state_label_;
};

}  // namespace rc26_xhu_viewer

#endif  // RC26_XHU_VIEWER__PANELS__NAVIGATION_PANEL_HPP_
