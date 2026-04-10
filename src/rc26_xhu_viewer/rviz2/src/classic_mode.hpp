#ifndef RVIZ2__CLASSIC_MODE_HPP_
#define RVIZ2__CLASSIC_MODE_HPP_

#include <vector>
#include <string>

class QApplication;

namespace rviz2
{

int runClassicMode(QApplication & qapp, std::vector<std::string> & args);

}  // namespace rviz2

#endif  // RVIZ2__CLASSIC_MODE_HPP_
