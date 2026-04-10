#ifndef RC26_XHU_VIEWER__DISPLAYS__DYNAMIC_PREDICTION_DISPLAY_HPP_
#define RC26_XHU_VIEWER__DISPLAYS__DYNAMIC_PREDICTION_DISPLAY_HPP_

#include <rviz_common/display.hpp>
#include <rviz_common/properties/string_property.hpp>

namespace rc26_xhu_viewer
{

/// Status overlay for dynamic obstacle prediction visualization.
/// Subscribes (when source modules publish):
///   rc26_interfaces/msg/DynamicPredictionArray
class DynamicPredictionDisplay : public rviz_common::Display
{
  Q_OBJECT

public:
  DynamicPredictionDisplay();

  void onInitialize() override;
  void update(float wall_dt, float ros_dt) override;
  void reset() override;

private:
  rviz_common::properties::StringProperty * prediction_status_;
};

}  // namespace rc26_xhu_viewer

#endif  // RC26_XHU_VIEWER__DISPLAYS__DYNAMIC_PREDICTION_DISPLAY_HPP_
