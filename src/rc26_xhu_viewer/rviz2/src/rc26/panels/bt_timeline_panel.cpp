#include "bt_timeline_panel.hpp"

#include <QLabel>
#include <QVBoxLayout>

namespace rc26_xhu_viewer
{

BtTimelinePanel::BtTimelinePanel(QWidget * parent)
: rviz_common::Panel(parent)
{
  auto * layout = new QVBoxLayout(this);

  event_count_label_ = new QLabel(QStringLiteral("时间线事件数: 等待数据..."));
  latest_event_label_ = new QLabel(QStringLiteral("最新事件: 等待数据..."));

  layout->addWidget(event_count_label_);
  layout->addWidget(latest_event_label_);
}

void BtTimelinePanel::load(const rviz_common::Config & config)
{
  rviz_common::Panel::load(config);
}

void BtTimelinePanel::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);
  config.mapSetValue("Class", "rviz2_rc26/BtTimelinePanel");
}

}  // namespace rc26_xhu_viewer

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rc26_xhu_viewer::BtTimelinePanel, rviz_common::Panel)
