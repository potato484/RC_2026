#ifndef RC26_XHU_VIEWER__PANELS__SERIAL_PANEL_HPP_
#define RC26_XHU_VIEWER__PANELS__SERIAL_PANEL_HPP_

#include <rviz_common/panel.hpp>

class QLabel;
class QTextEdit;

namespace rc26_xhu_viewer
{

class SerialPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit SerialPanel(QWidget * parent = nullptr);

  void load(const rviz_common::Config & config) override;
  void save(rviz_common::Config config) const override;

private:
  QLabel * title_label_{nullptr};
  QTextEdit * frame_log_{nullptr};
};

}  // namespace rc26_xhu_viewer

#endif  // RC26_XHU_VIEWER__PANELS__SERIAL_PANEL_HPP_
