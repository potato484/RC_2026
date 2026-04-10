#ifndef RVIZ2__RC26__RC26_MODE_HPP_
#define RVIZ2__RC26__RC26_MODE_HPP_

#include <string>
#include <vector>

class QApplication;

namespace rviz2
{

int runRc26Mode(
  QApplication & qapp, std::vector<std::string> & args, const std::string & mode,
  const std::string & layout);

}  // namespace rviz2

#endif  // RVIZ2__RC26__RC26_MODE_HPP_
