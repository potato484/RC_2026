#include "classic_mode.hpp"

#include <memory>
#include <string>
#include <vector>

#include <QApplication>  // NOLINT: cpplint is unable to handle include order here

#include "rclcpp/rclcpp.hpp"
#include "rviz_common/ros_integration/ros_client_abstraction.hpp"
#include "rviz_common/visualizer_app.hpp"

#include "mode_utils.hpp"

namespace rviz2
{

int runClassicMode(QApplication & qapp, std::vector<std::string> & args)
{
  auto logger = rclcpp::get_logger("rviz2");
  installRosLoggingHandlers(logger);

  auto argv_chars = toMutableArgv(args);
  int filtered_argc = static_cast<int>(argv_chars.size());
  rviz_common::VisualizerApp vapp(
    std::make_unique<rviz_common::ros_integration::RosClientAbstraction>());
  vapp.setApp(&qapp);
  if (vapp.init(filtered_argc, argv_chars.data())) {
    return qapp.exec();
  }
  return 1;
}

}  // namespace rviz2
