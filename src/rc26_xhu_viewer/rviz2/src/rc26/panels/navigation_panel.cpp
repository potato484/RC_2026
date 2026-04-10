#include "navigation_panel.hpp"

#include <QLabel>
#include <QVBoxLayout>

namespace rc26_xhu_viewer
{

NavigationPanel::NavigationPanel(QWidget * parent)
: rviz_common::Panel(parent)
{
  auto * layout = new QVBoxLayout(this);

  motion_mode_label_ = new QLabel(QStringLiteral("运动模式: 等待数据..."));
  tracking_state_label_ = new QLabel(QStringLiteral("跟踪状态: 等待数据..."));
  local_planner_label_ = new QLabel(QStringLiteral("局部规划状态: 等待数据..."));
  recovery_state_label_ = new QLabel(QStringLiteral("恢复状态: 等待数据..."));

  layout->addWidget(motion_mode_label_);
  layout->addWidget(tracking_state_label_);
  layout->addWidget(local_planner_label_);
  layout->addWidget(recovery_state_label_);
}

void NavigationPanel::load(const rviz_common::Config & config)
{
  rviz_common::Panel::load(config);
}

void NavigationPanel::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);
  config.mapSetValue("Class", "rviz2_rc26/NavigationPanel");
}

}  // namespace rc26_xhu_viewer

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rc26_xhu_viewer::NavigationPanel, rviz_common::Panel)
