#include "mechanism_panel.hpp"

#include <QLabel>
#include <QVBoxLayout>

namespace rc26_xhu_viewer
{

MechanismPanel::MechanismPanel(QWidget * parent)
: rviz_common::Panel(parent)
{
  auto * layout = new QVBoxLayout(this);

  state_label_ = new QLabel(QStringLiteral("机构状态: 等待数据..."));
  recent_action_label_ = new QLabel(QStringLiteral("最近动作: 等待数据..."));
  action_history_count_label_ = new QLabel(QStringLiteral("动作历史数: 等待数据..."));

  layout->addWidget(state_label_);
  layout->addWidget(recent_action_label_);
  layout->addWidget(action_history_count_label_);
}

void MechanismPanel::load(const rviz_common::Config & config)
{
  rviz_common::Panel::load(config);
}

void MechanismPanel::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);
  config.mapSetValue("Class", "rc26_xhu_viewer/MechanismPanel");
}

}  // namespace rc26_xhu_viewer

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rc26_xhu_viewer::MechanismPanel, rviz_common::Panel)
