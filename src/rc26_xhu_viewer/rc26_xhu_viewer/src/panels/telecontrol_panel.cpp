#include "telecontrol_panel.hpp"

#include <QLabel>
#include <QVBoxLayout>

namespace rc26_xhu_viewer
{

TelecontrolPanel::TelecontrolPanel(QWidget * parent)
: rviz_common::Panel(parent)
{
  auto * layout = new QVBoxLayout(this);

  linear_vel_label_ = new QLabel(QStringLiteral("线速度: 等待数据..."));
  angular_vel_label_ = new QLabel(QStringLiteral("角速度: 等待数据..."));
  controller_status_label_ = new QLabel(QStringLiteral("遥控器状态: 等待数据..."));

  layout->addWidget(linear_vel_label_);
  layout->addWidget(angular_vel_label_);
  layout->addWidget(controller_status_label_);
}

void TelecontrolPanel::load(const rviz_common::Config & config)
{
  rviz_common::Panel::load(config);
}

void TelecontrolPanel::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);
  config.mapSetValue("Class", "rc26_xhu_viewer/TelecontrolPanel");
}

}  // namespace rc26_xhu_viewer

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rc26_xhu_viewer::TelecontrolPanel, rviz_common::Panel)
