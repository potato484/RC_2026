#ifndef RC26_XHU_VIEWER__DISPLAYS__LOCALIZATION_DISPLAY_HPP_
#define RC26_XHU_VIEWER__DISPLAYS__LOCALIZATION_DISPLAY_HPP_

#include <rviz_common/display.hpp>
#include <rviz_common/properties/string_property.hpp>

namespace rc26_xhu_viewer
{

/// Status overlay for localization pipeline.
/// Subscribes (when source modules publish):
///   /localization/pose_with_covariance  (geometry_msgs/PoseWithCovarianceStamped)
///   keyframe array                       (rc26_interfaces/msg/LocalizationKeyframe)
///   loop closure array                   (rc26_interfaces/msg/LocalizationLoopClosure)
/// Future 3D rendering: covariance ellipsoid, trajectory, keyframe markers, loop closure lines.
class LocalizationDisplay : public rviz_common::Display
{
  Q_OBJECT

public:
  LocalizationDisplay();

  void onInitialize() override;
  void update(float wall_dt, float ros_dt) override;
  void reset() override;

private:
  rviz_common::properties::StringProperty * pose_status_;
  rviz_common::properties::StringProperty * keyframe_status_;
  rviz_common::properties::StringProperty * loop_closure_status_;
};

}  // namespace rc26_xhu_viewer

#endif  // RC26_XHU_VIEWER__DISPLAYS__LOCALIZATION_DISPLAY_HPP_
