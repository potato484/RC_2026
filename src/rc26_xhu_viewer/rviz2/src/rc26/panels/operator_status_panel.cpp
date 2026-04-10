#include "operator_status_panel.hpp"

#include <QLabel>
#include <QVBoxLayout>

namespace rc26_xhu_viewer
{

OperatorStatusPanel::OperatorStatusPanel(QWidget * parent)
: rviz_common::Panel(parent)
{
  auto * layout = new QVBoxLayout(this);

  operator_status_label_ = new QLabel(QStringLiteral("操作员状态: 等待数据..."));
  diag_event_count_label_ = new QLabel(QStringLiteral("诊断事件数: 等待数据..."));
  recent_event_label_ = new QLabel(QStringLiteral("最近事件: 等待数据..."));

  layout->addWidget(operator_status_label_);
  layout->addWidget(diag_event_count_label_);
  layout->addWidget(recent_event_label_);
}

void OperatorStatusPanel::load(const rviz_common::Config & config)
{
  rviz_common::Panel::load(config);
}

void OperatorStatusPanel::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);
  config.mapSetValue("Class", "rviz2_rc26/OperatorStatusPanel");
}

}  // namespace rc26_xhu_viewer

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rc26_xhu_viewer::OperatorStatusPanel, rviz_common::Panel)
