#include "lio_panel.hpp"

#include <QLabel>
#include <QVBoxLayout>

namespace rc26_xhu_viewer
{

LioPanel::LioPanel(QWidget * parent)
: rviz_common::Panel(parent)
{
  auto * layout = new QVBoxLayout(this);

  degenerate_score_label_ = new QLabel(QStringLiteral("退化评分: 等待数据..."));
  control_state_label_ = new QLabel(QStringLiteral("控制状态: 等待数据..."));

  layout->addWidget(degenerate_score_label_);
  layout->addWidget(control_state_label_);
}

void LioPanel::load(const rviz_common::Config & config)
{
  rviz_common::Panel::load(config);
}

void LioPanel::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);
  config.mapSetValue("Class", "rc26_xhu_viewer/LioPanel");
}

}  // namespace rc26_xhu_viewer

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rc26_xhu_viewer::LioPanel, rviz_common::Panel)
