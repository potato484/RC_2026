#ifndef RVIZ2__LAUNCH_MODE_HPP_
#define RVIZ2__LAUNCH_MODE_HPP_

#include <string>
#include <vector>

namespace rviz2
{

struct LaunchMode
{
  bool classic{false};
  std::string rc26_mode{"navigation"};
  std::string rc26_layout{"operator"};
  std::vector<std::string> filtered_args;
};

LaunchMode parseLaunchMode(int argc, char ** argv);
bool hasDisplayConfig(const std::vector<std::string> & args);

}  // namespace rviz2

#endif  // RVIZ2__LAUNCH_MODE_HPP_
