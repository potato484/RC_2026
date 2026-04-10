#include "localization_panel.hpp"

#include <QLabel>
#include <QVBoxLayout>

namespace rc26_xhu_viewer
{

LocalizationPanel::LocalizationPanel(QWidget * parent)
: rviz_common::Panel(parent)
{
  auto * layout = new QVBoxLayout(this);

  health_label_ = new QLabel(QStringLiteral("定位健康度: 等待数据..."));
  backend_status_label_ = new QLabel(QStringLiteral("后端状态: 等待数据..."));
  route_observability_label_ = new QLabel(QStringLiteral("路线可观性: 等待数据..."));
  reloc_state_label_ = new QLabel(QStringLiteral("重定位状态: 等待数据..."));

  layout->addWidget(health_label_);
  layout->addWidget(backend_status_label_);
  layout->addWidget(route_observability_label_);
  layout->addWidget(reloc_state_label_);
}

void LocalizationPanel::load(const rviz_common::Config & config)
{
  rviz_common::Panel::load(config);
}

void LocalizationPanel::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);
  config.mapSetValue("Class", "rviz2_rc26/LocalizationPanel");
}

}  // namespace rc26_xhu_viewer

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rc26_xhu_viewer::LocalizationPanel, rviz_common::Panel)
