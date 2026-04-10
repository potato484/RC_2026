#include "terrain_panel.hpp"

#include <QLabel>
#include <QVBoxLayout>

namespace rc26_xhu_viewer
{

TerrainPanel::TerrainPanel(QWidget * parent)
: rviz_common::Panel(parent)
{
  auto * layout = new QVBoxLayout(this);

  semantic_layer_label_ = new QLabel(QStringLiteral("语义层摘要: 等待数据..."));
  terrain_features_label_ = new QLabel(QStringLiteral("地形特征: 等待数据..."));

  layout->addWidget(semantic_layer_label_);
  layout->addWidget(terrain_features_label_);
}

void TerrainPanel::load(const rviz_common::Config & config)
{
  rviz_common::Panel::load(config);
}

void TerrainPanel::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);
  config.mapSetValue("Class", "rviz2_rc26/TerrainPanel");
}

}  // namespace rc26_xhu_viewer

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rc26_xhu_viewer::TerrainPanel, rviz_common::Panel)
