#include "bt_area_panel.hpp"

#include <QLabel>
#include <QVBoxLayout>

namespace rc26_xhu_viewer
{

BtAreaPanel::BtAreaPanel(QWidget * parent)
: rviz_common::Panel(parent)
{
  auto * layout = new QVBoxLayout(this);

  current_area_label_ = new QLabel(QStringLiteral("当前区域: 等待数据..."));
  field_state_label_ = new QLabel(QStringLiteral("场地状态: 等待数据..."));

  layout->addWidget(current_area_label_);
  layout->addWidget(field_state_label_);
}

void BtAreaPanel::load(const rviz_common::Config & config)
{
  rviz_common::Panel::load(config);
}

void BtAreaPanel::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);
  config.mapSetValue("Class", "rviz2_rc26/BtAreaPanel");
}

}  // namespace rc26_xhu_viewer

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rc26_xhu_viewer::BtAreaPanel, rviz_common::Panel)
