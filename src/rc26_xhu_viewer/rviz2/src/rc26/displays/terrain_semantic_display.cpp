#include "terrain_semantic_display.hpp"

#include <rviz_common/display_context.hpp>

namespace rc26_xhu_viewer
{

TerrainSemanticDisplay::TerrainSemanticDisplay()
{
  grid_status_ = new rviz_common::properties::StringProperty(
    "GridMap", QString::fromUtf8("等待数据..."),
    QString::fromUtf8("地形语义栅格 GridMap 接收状态"), this);
}

void TerrainSemanticDisplay::onInitialize()
{
  Display::onInitialize();
}

void TerrainSemanticDisplay::update(float /*wall_dt*/, float /*ros_dt*/)
{
  // future: check subscription freshness and update status text
}

void TerrainSemanticDisplay::reset()
{
  Display::reset();
  grid_status_->setValue(QString::fromUtf8("等待数据..."));
}

}  // namespace rc26_xhu_viewer

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  rc26_xhu_viewer::TerrainSemanticDisplay, rviz_common::Display)
