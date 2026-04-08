#ifndef RC26_XHU_VIEWER__DISPLAYS__NAV_CANDIDATES_DISPLAY_HPP_
#define RC26_XHU_VIEWER__DISPLAYS__NAV_CANDIDATES_DISPLAY_HPP_

#include <rviz_common/display.hpp>
#include <rviz_common/properties/string_property.hpp>

namespace rc26_xhu_viewer
{

/// Status overlay for navigation candidate visualization.
/// Subscribes (when source modules publish):
///   /xhu_nav/local_candidates   (visualization_msgs/MarkerArray)
///   /xhu_nav/selected_trajectory (nav_msgs/Path)
class NavCandidatesDisplay : public rviz_common::Display
{
  Q_OBJECT

public:
  NavCandidatesDisplay();

  void onInitialize() override;
  void update(float wall_dt, float ros_dt) override;
  void reset() override;

private:
  rviz_common::properties::StringProperty * candidates_status_;
  rviz_common::properties::StringProperty * trajectory_status_;
};

}  // namespace rc26_xhu_viewer

#endif  // RC26_XHU_VIEWER__DISPLAYS__NAV_CANDIDATES_DISPLAY_HPP_
