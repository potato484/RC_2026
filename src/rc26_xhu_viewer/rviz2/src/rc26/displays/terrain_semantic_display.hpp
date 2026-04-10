#ifndef RC26_XHU_VIEWER__DISPLAYS__TERRAIN_SEMANTIC_DISPLAY_HPP_
#define RC26_XHU_VIEWER__DISPLAYS__TERRAIN_SEMANTIC_DISPLAY_HPP_

#include <rviz_common/display.hpp>
#include <rviz_common/properties/string_property.hpp>

namespace rc26_xhu_viewer
{

/// Status overlay for terrain semantic grid visualization.
/// Subscribes (when source modules publish):
///   terrain_grid_map (grid_map_msgs/GridMap)
class TerrainSemanticDisplay : public rviz_common::Display
{
  Q_OBJECT

public:
  TerrainSemanticDisplay();

  void onInitialize() override;
  void update(float wall_dt, float ros_dt) override;
  void reset() override;

private:
  rviz_common::properties::StringProperty * grid_status_;
};

}  // namespace rc26_xhu_viewer

#endif  // RC26_XHU_VIEWER__DISPLAYS__TERRAIN_SEMANTIC_DISPLAY_HPP_
