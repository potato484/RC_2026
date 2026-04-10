#ifndef RC26_XHU_VIEWER__PANELS__VISION_PANEL_HPP_
#define RC26_XHU_VIEWER__PANELS__VISION_PANEL_HPP_

#include <rviz_common/panel.hpp>

class QLabel;

namespace rc26_xhu_viewer
{

class VisionPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit VisionPanel(QWidget * parent = nullptr);

  void load(const rviz_common::Config & config) override;
  void save(rviz_common::Config config) const override;

private:
  QLabel * detection_count_label_{nullptr};
  QLabel * recent_detection_label_{nullptr};
};

}  // namespace rc26_xhu_viewer

#endif  // RC26_XHU_VIEWER__PANELS__VISION_PANEL_HPP_
