#ifndef RC26_XHU_VIEWER__PANELS__LIO_PANEL_HPP_
#define RC26_XHU_VIEWER__PANELS__LIO_PANEL_HPP_

#include <rviz_common/panel.hpp>

class QLabel;

namespace rc26_xhu_viewer
{

class LioPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit LioPanel(QWidget * parent = nullptr);

  void load(const rviz_common::Config & config) override;
  void save(rviz_common::Config config) const override;

private:
  QLabel * degenerate_score_label_;
  QLabel * control_state_label_;
};

}  // namespace rc26_xhu_viewer

#endif  // RC26_XHU_VIEWER__PANELS__LIO_PANEL_HPP_
