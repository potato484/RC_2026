#ifndef RC26_XHU_VIEWER__PANELS__MECHANISM_PANEL_HPP_
#define RC26_XHU_VIEWER__PANELS__MECHANISM_PANEL_HPP_

#include <rviz_common/panel.hpp>

class QLabel;

namespace rc26_xhu_viewer
{

class MechanismPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit MechanismPanel(QWidget * parent = nullptr);

  void load(const rviz_common::Config & config) override;
  void save(rviz_common::Config config) const override;

private:
  QLabel * state_label_{nullptr};
  QLabel * recent_action_label_{nullptr};
  QLabel * action_history_count_label_{nullptr};
};

}  // namespace rc26_xhu_viewer

#endif  // RC26_XHU_VIEWER__PANELS__MECHANISM_PANEL_HPP_
