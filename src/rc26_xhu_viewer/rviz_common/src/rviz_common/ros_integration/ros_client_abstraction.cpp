/*
 * Copyright (c) 2017, Open Source Robotics Foundation, Inc.
 * Copyright (c) 2017, Bosch Software Innovations GmbH.
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

#include "rviz_common/ros_integration/ros_client_abstraction.hpp"

#include <csignal>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "rviz_common/ros_integration/ros_node_abstraction.hpp"

namespace rviz_common
{
namespace ros_integration
{

namespace
{

volatile std::sig_atomic_t g_shutdown_signal = 0;
using SignalHandler = void (*)(int);
SignalHandler g_previous_sigint_handler = SIG_DFL;
SignalHandler g_previous_sigterm_handler = SIG_DFL;
bool g_signal_handlers_installed = false;
std::vector<std::shared_ptr<RosNodeAbstractionIface>> * g_leaked_signal_exit_nodes = nullptr;

void rvizSignalHandler(int signum)
{
  g_shutdown_signal = signum;
}

void installSignalHandlers()
{
  if (g_signal_handlers_installed) {
    g_shutdown_signal = 0;
    return;
  }

  g_previous_sigint_handler = std::signal(SIGINT, rvizSignalHandler);
  g_previous_sigterm_handler = std::signal(SIGTERM, rvizSignalHandler);
  g_shutdown_signal = 0;
  g_signal_handlers_installed = true;
}

void restoreSignalHandlers()
{
  if (!g_signal_handlers_installed) {
    g_shutdown_signal = 0;
    return;
  }

  std::signal(SIGINT, g_previous_sigint_handler);
  std::signal(SIGTERM, g_previous_sigterm_handler);
  g_shutdown_signal = 0;
  g_signal_handlers_installed = false;
}

}  // namespace

RosClientAbstraction::RosClientAbstraction()
{}

RosNodeAbstractionIface::WeakPtr
RosClientAbstraction::init(int argc, char ** argv, const std::string & name, bool anonymous_name)
{
  std::string final_name = name;
  if (anonymous_name) {
    // TODO(wjwwood): add anonymous name feature to rclcpp or somehow make name
    //                anonymouse here.
    throw std::runtime_error("'anonymous_name' feature not implemented");
    // final_name = <the full anonymous node name>;
  }
  // RViz runs a Qt event loop and must destroy its frames and nodes before
  // ROS shutdown. The default rclcpp signal handler can shut the global
  // context down asynchronously, which then crashes node teardown on exit.
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  installSignalHandlers();
  if (rviz_ros_node_ && rviz_ros_node_->get_node_name() == final_name) {
    // TODO(wjwwood): make a better exception type rather than using std::runtime_error.
    throw std::runtime_error("Node with name " + final_name + " already exists.");
  }
  rviz_ros_node_ = std::make_shared<RosNodeAbstraction>(final_name);
  return rviz_ros_node_;
}

bool
RosClientAbstraction::ok()
{
  return g_shutdown_signal == 0 && rclcpp::ok() && rviz_ros_node_;
}

void
RosClientAbstraction::shutdown()
{
  const bool signal_requested = g_shutdown_signal != 0;
  restoreSignalHandlers();
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }

  if (signal_requested && rviz_ros_node_) {
    // Humble can still crash inside rclcpp::CallbackGroup teardown if a live
    // RViz node is destroyed on a process signal path after the Qt UI has
    // already started unwinding. Keep the node alive until process exit in
    // that specific path instead of crashing on Ctrl+C.
    if (!g_leaked_signal_exit_nodes) {
      g_leaked_signal_exit_nodes = new std::vector<std::shared_ptr<RosNodeAbstractionIface>>();
    }
    g_leaked_signal_exit_nodes->push_back(std::move(rviz_ros_node_));
    return;
  }

  rviz_ros_node_.reset();
}

}  // namespace ros_integration
}  // namespace rviz_common
