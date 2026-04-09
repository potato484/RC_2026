/*
 * Copyright (c) 2011, Willow Garage, Inc.
 * Copyright (c) 2017, Open Source Robotics Foundation, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <exception>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

#include <QApplication>  // NOLINT: cpplint is unable to handle include order here
#include <QCoreApplication>  // NOLINT: cpplint is unable to handle include order here
#include <QEvent>  // NOLINT: cpplint is unable to handle include order here
#include <QString>  // NOLINT: cpplint is unable to handle include order here
#include <QTimer>  // NOLINT: cpplint is unable to handle include order here

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rviz_common/logging.hpp"
#include "rviz_common/ros_integration/ros_client_abstraction.hpp"
#include "rviz_common/visualizer_app.hpp"
#include "rviz_rendering/ogre_logging.hpp"

#include "rc26_xhu_viewer_frame.hpp"

namespace
{

struct LaunchMode
{
  bool classic{false};
  std::string rc26_mode{"navigation"};
  std::string rc26_layout{"operator"};
  std::vector<std::string> filtered_args;
};

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

bool consumeFlag(
  const std::vector<std::string> & input_args,
  std::vector<std::string> & filtered_args,
  const std::string & option_name)
{
  bool enabled = false;
  for (size_t index = 1; index < input_args.size(); ++index) {
    const std::string & arg = input_args[index];
    if (arg == option_name) {
      enabled = true;
      continue;
    }
    filtered_args.push_back(arg);
  }
  return enabled;
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
  const std::string package_share = ament_index_cpp::get_package_share_directory("rviz2");
  return package_share + "/config/" + mode + "_" + layout + ".rviz";
}

LaunchMode parseLaunchMode(int argc, char ** argv)
{
  LaunchMode mode;
  std::vector<std::string> original_args;
  original_args.reserve(static_cast<size_t>(argc));
  for (int index = 0; index < argc; ++index) {
    original_args.emplace_back(argv[index]);
  }

  std::vector<std::string> without_classic;
  without_classic.push_back(original_args.front());
  mode.classic = consumeFlag(original_args, without_classic, "--classic");

  std::vector<std::string> without_layout;
  without_layout.push_back(without_classic.front());
  mode.rc26_layout = consumeOption(
    without_classic, without_layout, "--rc26-layout", "operator");
  std::vector<std::string> without_legacy_layout;
  without_legacy_layout.push_back(without_layout.front());
  mode.rc26_layout = consumeOption(
    without_layout, without_legacy_layout, "--layout", mode.rc26_layout);

  mode.filtered_args.push_back(without_legacy_layout.front());
  mode.rc26_mode = consumeOption(
    without_legacy_layout, mode.filtered_args, "--rc26-mode", "navigation");
  std::vector<std::string> without_legacy_mode;
  without_legacy_mode.push_back(mode.filtered_args.front());
  mode.rc26_mode = consumeOption(
    mode.filtered_args, without_legacy_mode, "--mode", mode.rc26_mode);
  mode.filtered_args = std::move(without_legacy_mode);

  return mode;
}

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
    }
  );
}

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

int runRc26Mode(QApplication & qapp, std::vector<std::string> & args, const std::string & mode,
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

  QString display_config;
  for (size_t i = 1; i < args.size(); ++i) {
    if ((args[i] == "-d" || args[i] == "--display-config") && i + 1 < args.size()) {
      display_config = QString::fromStdString(args[i + 1]);
      break;
    }
    if (args[i].rfind("--display-config=", 0) == 0) {
      display_config = QString::fromStdString(args[i].substr(std::string("--display-config=").size()));
      break;
    }
  }

  auto * frame = new rc26_xhu_viewer::RC26XhuViewerFrame(node);
  frame->setApp(&qapp);
  frame->initialize(node, display_config);
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

}  // namespace

int main(int argc, char ** argv)
{
  LaunchMode launch_mode = parseLaunchMode(argc, argv);

  auto filtered_arg_chars = toMutableArgv(launch_mode.filtered_args);
  std::vector<std::string> non_ros_qt_args =
    rclcpp::remove_ros_arguments(
      static_cast<int>(launch_mode.filtered_args.size()), filtered_arg_chars.data());
  auto non_ros_qt_arg_chars = toMutableArgv(non_ros_qt_args);
  int non_ros_qt_argc = static_cast<int>(non_ros_qt_arg_chars.size());

  QApplication::setApplicationName("rviz2");
  QApplication qapp(non_ros_qt_argc, non_ros_qt_arg_chars.data());

  if (launch_mode.classic) {
    return runClassicMode(qapp, launch_mode.filtered_args);
  }

  return runRc26Mode(
    qapp, launch_mode.filtered_args, launch_mode.rc26_mode, launch_mode.rc26_layout);
}
