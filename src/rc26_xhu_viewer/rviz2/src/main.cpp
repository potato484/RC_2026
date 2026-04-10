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

#include <string>
#include <vector>

#include <QApplication>  // NOLINT: cpplint is unable to handle include order here

#include "rclcpp/rclcpp.hpp"

#include "classic_mode.hpp"
#include "launch_mode.hpp"
#include "mode_utils.hpp"
#include "rc26/rc26_mode.hpp"

int main(int argc, char ** argv)
{
  rviz2::LaunchMode launch_mode = rviz2::parseLaunchMode(argc, argv);

  auto filtered_arg_chars = rviz2::toMutableArgv(launch_mode.filtered_args);
  std::vector<std::string> non_ros_qt_args =
    rclcpp::remove_ros_arguments(
      static_cast<int>(launch_mode.filtered_args.size()), filtered_arg_chars.data());
  auto non_ros_qt_arg_chars = rviz2::toMutableArgv(non_ros_qt_args);
  int non_ros_qt_argc = static_cast<int>(non_ros_qt_arg_chars.size());

  QApplication::setApplicationName("rviz2");
  QApplication qapp(non_ros_qt_argc, non_ros_qt_arg_chars.data());

  if (launch_mode.classic) {
    return rviz2::runClassicMode(qapp, launch_mode.filtered_args);
  }

  return rviz2::runRc26Mode(
    qapp, launch_mode.filtered_args, launch_mode.rc26_mode, launch_mode.rc26_layout);
}
