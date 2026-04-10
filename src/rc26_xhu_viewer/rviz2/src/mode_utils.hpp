#ifndef RVIZ2__MODE_UTILS_HPP_
#define RVIZ2__MODE_UTILS_HPP_

#include <string>
#include <vector>

#include <QString>

#include "rclcpp/logger.hpp"

namespace rviz2
{

std::vector<char *> toMutableArgv(std::vector<std::string> & args);
void installRosLoggingHandlers(const rclcpp::Logger & logger);
std::string resolveDefaultConfig(const std::string & mode, const std::string & layout);
QString extractDisplayConfig(const std::vector<std::string> & args);

}  // namespace rviz2

#endif  // RVIZ2__MODE_UTILS_HPP_
