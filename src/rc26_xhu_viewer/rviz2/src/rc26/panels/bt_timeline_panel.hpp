#ifndef RC26_XHU_VIEWER__PANELS__BT_TIMELINE_PANEL_HPP_
#define RC26_XHU_VIEWER__PANELS__BT_TIMELINE_PANEL_HPP_

#include <rviz_common/panel.hpp>

class QLabel;

namespace rc26_xhu_viewer
{

class BtTimelinePanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit BtTimelinePanel(QWidget * parent = nullptr);

  void load(const rviz_common::Config & config) override;
  void save(rviz_common::Config config) const override;

private:
  QLabel * event_count_label_{nullptr};
  QLabel * latest_event_label_{nullptr};
};

}  // namespace rc26_xhu_viewer

#endif  // RC26_XHU_VIEWER__PANELS__BT_TIMELINE_PANEL_HPP_
