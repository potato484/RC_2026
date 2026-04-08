#ifndef RC26_XHU_VIEWER__DISPLAYS__REGISTRATION_DEBUG_DISPLAY_HPP_
#define RC26_XHU_VIEWER__DISPLAYS__REGISTRATION_DEBUG_DISPLAY_HPP_

#include <rviz_common/display.hpp>
#include <rviz_common/properties/string_property.hpp>

namespace rc26_xhu_viewer
{

/// Status overlay for point cloud registration debug info.
/// Subscribes (when source modules publish):
///   rc26_interfaces/msg/RegistrationDebug
/// Shows: fitness_score, inlier_count, converged status.
class RegistrationDebugDisplay : public rviz_common::Display
{
  Q_OBJECT

public:
  RegistrationDebugDisplay();

  void onInitialize() override;
  void update(float wall_dt, float ros_dt) override;
  void reset() override;

private:
  rviz_common::properties::StringProperty * fitness_status_;
  rviz_common::properties::StringProperty * inlier_status_;
  rviz_common::properties::StringProperty * converged_status_;
};

}  // namespace rc26_xhu_viewer

#endif  // RC26_XHU_VIEWER__DISPLAYS__REGISTRATION_DEBUG_DISPLAY_HPP_
