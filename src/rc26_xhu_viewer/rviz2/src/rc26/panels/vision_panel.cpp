#include "vision_panel.hpp"

#include <QLabel>
#include <QVBoxLayout>

namespace rc26_xhu_viewer
{

VisionPanel::VisionPanel(QWidget * parent)
: rviz_common::Panel(parent)
{
  auto * layout = new QVBoxLayout(this);

  detection_count_label_ = new QLabel(QStringLiteral("检测数量: 等待数据..."));
  recent_detection_label_ = new QLabel(QStringLiteral("最近检测: 等待数据..."));

  layout->addWidget(detection_count_label_);
  layout->addWidget(recent_detection_label_);
}

void VisionPanel::load(const rviz_common::Config & config)
{
  rviz_common::Panel::load(config);
}

void VisionPanel::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);
  config.mapSetValue("Class", "rviz2_rc26/VisionPanel");
}

}  // namespace rc26_xhu_viewer

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rc26_xhu_viewer::VisionPanel, rviz_common::Panel)
