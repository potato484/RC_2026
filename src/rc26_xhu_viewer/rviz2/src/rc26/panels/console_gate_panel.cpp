#include "console_gate_panel.hpp"

#include <QHBoxLayout>
#include <QLabel>
#include <QMessageBox>
#include <QPushButton>
#include <QVBoxLayout>

namespace rc26_xhu_viewer
{

RC26ConsoleGatePanel::RC26ConsoleGatePanel(QWidget * parent)
: rviz_common::Panel(parent)
{
  auto * layout = new QVBoxLayout(this);

  status_label_ = new QLabel(QStringLiteral("当前态: 操作态（只读）"));
  status_label_->setStyleSheet("font-weight: bold; font-size: 14px;");
  layout->addWidget(status_label_);

  auto * btn_layout = new QHBoxLayout();

  engineering_btn_ = new QPushButton(QStringLiteral("解锁工程态"));
  diagnostic_btn_ = new QPushButton(QStringLiteral("解锁诊断态"));
  lock_btn_ = new QPushButton(QStringLiteral("锁定"));
  lock_btn_->setEnabled(false);

  btn_layout->addWidget(engineering_btn_);
  btn_layout->addWidget(diagnostic_btn_);
  btn_layout->addWidget(lock_btn_);
  layout->addLayout(btn_layout);

  connect(engineering_btn_, &QPushButton::clicked, this, &RC26ConsoleGatePanel::onEngineeringClicked);
  connect(diagnostic_btn_, &QPushButton::clicked, this, &RC26ConsoleGatePanel::onDiagnosticClicked);
  connect(lock_btn_, &QPushButton::clicked, this, &RC26ConsoleGatePanel::onLockClicked);
}

void RC26ConsoleGatePanel::onEngineeringClicked()
{
  auto reply = QMessageBox::question(
    this,
    QStringLiteral("确认"),
    QStringLiteral("即将进入工程态，可调用运动模式切换、BT 控制和诊断重置服务。确认？"),
    QMessageBox::Yes | QMessageBox::No);
  if (reply == QMessageBox::Yes) {
    applyLevel(GateLevel::Engineering);
  }
}

void RC26ConsoleGatePanel::onDiagnosticClicked()
{
  if (level_ < GateLevel::Engineering) {
    QMessageBox::warning(
      this,
      QStringLiteral("权限不足"),
      QStringLiteral("请先解锁工程态。"));
    return;
  }
  auto reply = QMessageBox::warning(
    this,
    QStringLiteral("二次确认"),
    QStringLiteral("即将进入诊断态，可发送机构 transport 命令。此操作可能影响机器人硬件状态，是否继续？"),
    QMessageBox::Yes | QMessageBox::No);
  if (reply == QMessageBox::Yes) {
    applyLevel(GateLevel::Diagnostic);
  }
}

void RC26ConsoleGatePanel::onLockClicked()
{
  applyLevel(GateLevel::Operator);
}

void RC26ConsoleGatePanel::applyLevel(GateLevel level)
{
  level_ = level;
  switch (level_) {
    case GateLevel::Operator:
      status_label_->setText(QStringLiteral("当前态: 操作态（只读）"));
      status_label_->setStyleSheet("font-weight: bold; font-size: 14px; color: #2196F3;");
      engineering_btn_->setEnabled(true);
      diagnostic_btn_->setEnabled(true);
      lock_btn_->setEnabled(false);
      break;
    case GateLevel::Engineering:
      status_label_->setText(QStringLiteral("当前态: 工程态"));
      status_label_->setStyleSheet("font-weight: bold; font-size: 14px; color: #FF9800;");
      engineering_btn_->setEnabled(false);
      diagnostic_btn_->setEnabled(true);
      lock_btn_->setEnabled(true);
      break;
    case GateLevel::Diagnostic:
      status_label_->setText(QStringLiteral("当前态: 诊断态（危险）"));
      status_label_->setStyleSheet("font-weight: bold; font-size: 14px; color: #F44336;");
      engineering_btn_->setEnabled(false);
      diagnostic_btn_->setEnabled(false);
      lock_btn_->setEnabled(true);
      break;
  }
  Q_EMIT gateLevelChanged(static_cast<int>(level_));
}

void RC26ConsoleGatePanel::load(const rviz_common::Config & config)
{
  rviz_common::Panel::load(config);
}

void RC26ConsoleGatePanel::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);
  config.mapSetValue("Class", "rviz2_rc26/RC26ConsoleGatePanel");
}

}  // namespace rc26_xhu_viewer

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rc26_xhu_viewer::RC26ConsoleGatePanel, rviz_common::Panel)
