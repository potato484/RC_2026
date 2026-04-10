#ifndef RC26_XHU_VIEWER__PANELS__BT_AREA_PANEL_HPP_
#define RC26_XHU_VIEWER__PANELS__BT_AREA_PANEL_HPP_

#include <rviz_common/panel.hpp>

class QLabel;

namespace rc26_xhu_viewer
{

class BtAreaPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit BtAreaPanel(QWidget * parent = nullptr);

  void load(const rviz_common::Config & config) override;
  void save(rviz_common::Config config) const override;

private:
  QLabel * current_area_label_;
  QLabel * field_state_label_;
};

}  // namespace rc26_xhu_viewer

#endif  // RC26_XHU_VIEWER__PANELS__BT_AREA_PANEL_HPP_
