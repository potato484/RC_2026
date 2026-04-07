#include <filesystem>
#include <memory>
#include <string>
#include <vector>

#include <QApplication>
#include <QString>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rviz_common/logging.hpp"
#include "rviz_common/ros_integration/ros_client_abstraction.hpp"
#include "rviz_common/visualizer_app.hpp"

namespace
{

std::string consumeOption(
  const std::vector<std::string> & input_args,
  std::vector<std::string> & filtered_args,
  const std::string & option_name,
  const std::string & fallback_value)
{
  std::string value = fallback_value;
  for (size_t index = 1; index < input_args.size(); ++index) {
    const std::string & arg = input_args[index];
    if (arg == option_name) {
      if (index + 1 < input_args.size()) {
        value = input_args[index + 1];
        ++index;
      }
      continue;
    }
    if (arg.rfind(option_name + "=", 0) == 0) {
      value = arg.substr(option_name.size() + 1);
      continue;
    }
    filtered_args.push_back(arg);
  }
  return value;
}

bool hasDisplayConfig(const std::vector<std::string> & args)
{
  for (size_t index = 1; index < args.size(); ++index) {
    const std::string & arg = args[index];
    if (arg == "-d" || arg == "--display-config" || arg.rfind("--display-config=", 0) == 0) {
      return true;
    }
  }
  return false;
}

std::string resolveDefaultConfig(const std::string & mode, const std::string & layout)
{
  const std::string package_share =
    ament_index_cpp::get_package_share_directory("rc26_xhu_viewer");
  return package_share + "/config/" + mode + "_" + layout + ".rviz";
}

}  // namespace

int main(int argc, char ** argv)
{
  std::vector<std::string> original_args;
  original_args.reserve(static_cast<size_t>(argc));
  for (int index = 0; index < argc; ++index) {
    original_args.emplace_back(argv[index]);
  }

  std::vector<std::string> without_layout_flag;
  without_layout_flag.push_back(original_args.front());
  const std::string layout =
    consumeOption(original_args, without_layout_flag, "--layout", "operator");

  std::vector<std::string> filtered_args;
  filtered_args.push_back(without_layout_flag.front());
  const std::string mode =
    consumeOption(without_layout_flag, filtered_args, "--mode", "navigation");

  if (!hasDisplayConfig(filtered_args)) {
    try {
      const std::string config_path = resolveDefaultConfig(mode, layout);
      if (!std::filesystem::exists(config_path)) {
        RCLCPP_ERROR(
          rclcpp::get_logger("rc26_xhu_viewer"),
          "default RViz config does not exist: %s", config_path.c_str());
        return 1;
      }
      filtered_args.push_back("-d");
      filtered_args.push_back(config_path);
    } catch (const std::exception & ex) {
      RCLCPP_ERROR(
        rclcpp::get_logger("rc26_xhu_viewer"),
        "failed to resolve default config for mode=%s layout=%s: %s",
        mode.c_str(), layout.c_str(), ex.what());
      return 1;
    }
  }

  std::vector<char *> filtered_arg_chars;
  filtered_arg_chars.reserve(filtered_args.size());
  for (auto & arg : filtered_args) {
    filtered_arg_chars.push_back(arg.data());
  }

  int filtered_argc = static_cast<int>(filtered_arg_chars.size());
  std::vector<std::string> non_ros_args = rclcpp::remove_ros_arguments(
    filtered_argc, filtered_arg_chars.data());
  std::vector<char *> non_ros_arg_chars;
  non_ros_arg_chars.reserve(non_ros_args.size());
  for (auto & arg : non_ros_args) {
    non_ros_arg_chars.push_back(arg.data());
  }
  int non_ros_argc = static_cast<int>(non_ros_arg_chars.size());

  QApplication::setApplicationName("rc26_xhu_viewer");
  QApplication::setApplicationDisplayName(QStringLiteral("RC26 工程可视化台"));
  QApplication qapp(non_ros_argc, non_ros_arg_chars.data());

  const auto logger = rclcpp::get_logger("rc26_xhu_viewer");
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

  rviz_common::VisualizerApp viewer_app(
    std::make_unique<rviz_common::ros_integration::RosClientAbstraction>());
  viewer_app.setApp(&qapp);
  if (!viewer_app.init(filtered_argc, filtered_arg_chars.data())) {
    return 1;
  }
  return qapp.exec();
}
