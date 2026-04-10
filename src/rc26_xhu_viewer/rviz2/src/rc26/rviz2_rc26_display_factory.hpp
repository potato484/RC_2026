#ifndef RVIZ2__RC26__RVIZ2_RC26_DISPLAY_FACTORY_HPP_
#define RVIZ2__RC26__RVIZ2_RC26_DISPLAY_FACTORY_HPP_

#include <set>
#include <string>
#include <vector>

#include <QString>

#include "rviz_common/display.hpp"
#include "rviz_common/display_factory.hpp"
#include "rviz_common/logging.hpp"

namespace rviz2_rc26
{

class RViz2Rc26DisplayFactory : public rviz_common::DisplayFactory
{
public:
  RViz2Rc26DisplayFactory()
  : rviz_common::DisplayFactory()
  {
    allowed_ = {
      "rviz_common/Group",
      "rviz_default_plugins/Grid",
      "rviz_default_plugins/RobotModel",
      "rviz_default_plugins/Path",
      "rviz_default_plugins/PointCloud2",
      "rviz_default_plugins/PointStamped",
      "rviz_default_plugins/MarkerArray",
      "rviz_default_plugins/Map",
      "rviz_default_plugins/TF",
      "rviz_default_plugins/Marker",
      "rviz_default_plugins/PoseWithCovariance",
      "rviz_default_plugins/Odometry",
      "grid_map_rviz_plugin/GridMap",
      "rviz2_rc26/LocalizationDisplay",
      "rviz2_rc26/RegistrationDebugDisplay",
      "rviz2_rc26/NavCandidatesDisplay",
      "rviz2_rc26/DynamicPredictionDisplay",
      "rviz2_rc26/TerrainSemanticDisplay",
    };
  }

  std::vector<rviz_common::PluginInfo> getDeclaredPlugins() override
  {
    auto all = rviz_common::DisplayFactory::getDeclaredPlugins();
    std::vector<rviz_common::PluginInfo> filtered;
    for (auto & info : all) {
      if (allowed_.count(info.id.toStdString())) {
        filtered.push_back(info);
      }
    }
    return filtered;
  }

protected:
  rviz_common::Display * makeRaw(
    const QString & class_id, QString * error_return = nullptr) override
  {
    if (!allowed_.count(class_id.toStdString())) {
      RVIZ_COMMON_LOG_WARNING_STREAM(
        "RViz2Rc26DisplayFactory: blocked non-allowlisted plugin '" <<
          class_id.toStdString() << "'");
      if (error_return) {
        *error_return = "Plugin '" + class_id + "' is not in the RC26 allowlist.";
      }
      return nullptr;
    }
    return rviz_common::DisplayFactory::makeRaw(class_id, error_return);
  }

private:
  std::set<std::string> allowed_;
};

}  // namespace rviz2_rc26

#endif  // RVIZ2__RC26__RVIZ2_RC26_DISPLAY_FACTORY_HPP_
