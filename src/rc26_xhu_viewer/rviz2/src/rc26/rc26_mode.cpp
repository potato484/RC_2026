#include "rc26/rc26_mode.hpp"

#include <exception>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

#include <QApplication>  // NOLINT: cpplint is unable to handle include order here
#include <QCoreApplication>  // NOLINT: cpplint is unable to handle include order here
#include <QEvent>  // NOLINT: cpplint is unable to handle include order here
#include <QTimer>  // NOLINT: cpplint is unable to handle include order here

#include "rclcpp/rclcpp.hpp"
#include "rviz_common/logging.hpp"
#include "rviz_common/ros_integration/ros_client_abstraction.hpp"
#include "rviz_rendering/ogre_logging.hpp"

#include "launch_mode.hpp"
#include "mode_utils.hpp"
#include "rc26/rviz2_rc26_viewer_frame.hpp"

namespace rviz2
{

int runRc26Mode(
  QApplication & qapp, std::vector<std::string> & args, const std::string & mode,
  const std::string & layout)
{
  auto logger = rclcpp::get_logger("rviz2");
  installRosLoggingHandlers(logger);

  if (!hasDisplayConfig(args)) {
    try {
      const std::string config_path = resolveDefaultConfig(mode, layout);
      if (!std::filesystem::exists(config_path)) {
        RCLCPP_ERROR(logger, "default RViz config does not exist: %s", config_path.c_str());
        return 1;
      }
      args.push_back("-d");
      args.push_back(config_path);
    } catch (const std::exception & ex) {
      RCLCPP_ERROR(
        logger, "failed to resolve default config for rc26 mode=%s layout=%s: %s",
        mode.c_str(), layout.c_str(), ex.what());
      return 1;
    }
  }

  auto filtered_arg_chars = toMutableArgv(args);
  int filtered_argc = static_cast<int>(filtered_arg_chars.size());

  QApplication::setApplicationName("rviz2");
  QApplication::setApplicationDisplayName(QStringLiteral("RC26 工程可视化台"));
  qapp.setApplicationName("rviz2");
  qapp.setApplicationDisplayName(QStringLiteral("RC26 工程可视化台"));

  rviz_common::install_rviz_rendering_log_handlers();

  auto ros_client =
    std::make_unique<rviz_common::ros_integration::RosClientAbstraction>();
  auto node = ros_client->init(filtered_argc, filtered_arg_chars.data(), "rviz", false);

  auto * frame = new rviz2_rc26::RViz2Rc26ViewerFrame(node);
  frame->setApp(&qapp);
  frame->initialize(node, extractDisplayConfig(args));
  frame->show();

  auto * continue_timer = new QTimer(&qapp);
  QObject::connect(continue_timer, &QTimer::timeout, [&ros_client, frame]() {
    if (!ros_client->ok()) {
      frame->setWindowModified(false);
      QApplication::closeAllWindows();
    }
  });
  continue_timer->start(100);

  const int result = qapp.exec();
  continue_timer->stop();
  delete continue_timer;
  delete frame;
  QCoreApplication::sendPostedEvents(nullptr, QEvent::DeferredDelete);
  QCoreApplication::processEvents();
  ros_client->shutdown();
  return result;
}

}  // namespace rviz2
