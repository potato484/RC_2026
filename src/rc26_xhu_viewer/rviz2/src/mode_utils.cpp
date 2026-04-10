#include "mode_utils.hpp"

#include <string>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rviz_common/logging.hpp"

namespace rviz2
{

std::vector<char *> toMutableArgv(std::vector<std::string> & args)
{
  std::vector<char *> chars;
  chars.reserve(args.size());
  for (auto & arg : args) {
    chars.push_back(arg.data());
  }
  return chars;
}

void installRosLoggingHandlers(const rclcpp::Logger & logger)
{
  rviz_common::set_logging_handlers(
    [logger](const std::string & msg, const std::string &, size_t) {
      RCLCPP_DEBUG(logger, "%s", msg.c_str());
    },
    [logger](const std::string & msg, const std::string &, size_t) {
      RCLCPP_INFO(logger, "%s", msg.c_str());
    },
    [logger](const std::string & msg, const std::string &, size_t) {
      RCLCPP_WARN(logger, "%s", msg.c_str());
    },
    [logger](const std::string & msg, const std::string &, size_t) {
      RCLCPP_ERROR(logger, "%s", msg.c_str());
    });
}

std::string resolveDefaultConfig(const std::string & mode, const std::string & layout)
{
  const std::string package_share = ament_index_cpp::get_package_share_directory("rviz2");
  return package_share + "/config/" + mode + "_" + layout + ".rviz";
}

QString extractDisplayConfig(const std::vector<std::string> & args)
{
  QString display_config;
  for (size_t index = 1; index < args.size(); ++index) {
    if ((args[index] == "-d" || args[index] == "--display-config") && index + 1 < args.size()) {
      display_config = QString::fromStdString(args[index + 1]);
      break;
    }
    if (args[index].rfind("--display-config=", 0) == 0) {
      display_config = QString::fromStdString(
        args[index].substr(std::string("--display-config=").size()));
      break;
    }
  }
  return display_config;
}

}  // namespace rviz2
