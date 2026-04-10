#ifndef RC26_XHU_VIEWER__PANELS__CONSOLE_GATE_PANEL_HPP_
#define RC26_XHU_VIEWER__PANELS__CONSOLE_GATE_PANEL_HPP_

#include <rviz_common/panel.hpp>

class QLabel;
class QPushButton;

namespace rc26_xhu_viewer
{

enum class GateLevel : int
{
  Operator = 0,
  Engineering = 1,
  Diagnostic = 2,
};

class RC26ConsoleGatePanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit RC26ConsoleGatePanel(QWidget * parent = nullptr);

  void load(const rviz_common::Config & config) override;
  void save(rviz_common::Config config) const override;

  GateLevel currentLevel() const { return level_; }

Q_SIGNALS:
  void gateLevelChanged(int level);

private Q_SLOTS:
  void onEngineeringClicked();
  void onDiagnosticClicked();
  void onLockClicked();

private:
  void applyLevel(GateLevel level);

  GateLevel level_{GateLevel::Operator};
  QLabel * status_label_{nullptr};
  QPushButton * engineering_btn_{nullptr};
  QPushButton * diagnostic_btn_{nullptr};
  QPushButton * lock_btn_{nullptr};
};

}  // namespace rc26_xhu_viewer

#endif  // RC26_XHU_VIEWER__PANELS__CONSOLE_GATE_PANEL_HPP_
