#ifndef RC26_XHU_VIEWER__PANELS__TELECONTROL_PANEL_HPP_
#define RC26_XHU_VIEWER__PANELS__TELECONTROL_PANEL_HPP_

#include <rviz_common/panel.hpp>

class QLabel;

namespace rc26_xhu_viewer
{

class TelecontrolPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit TelecontrolPanel(QWidget * parent = nullptr);

  void load(const rviz_common::Config & config) override;
  void save(rviz_common::Config config) const override;

private:
  QLabel * linear_vel_label_{nullptr};
  QLabel * angular_vel_label_{nullptr};
  QLabel * controller_status_label_{nullptr};
};

}  // namespace rc26_xhu_viewer

#endif  // RC26_XHU_VIEWER__PANELS__TELECONTROL_PANEL_HPP_
