#ifndef RVIZ2__RC26__RVIZ2_RC26_VIEWER_FRAME_HPP_
#define RVIZ2__RC26__RVIZ2_RC26_VIEWER_FRAME_HPP_

#include "rviz_common/visualization_frame.hpp"

namespace rviz2_rc26
{

class RViz2Rc26ViewerFrame : public rviz_common::VisualizationFrame
{
  Q_OBJECT

public:
  explicit RViz2Rc26ViewerFrame(
    rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr rviz_ros_node,
    QWidget * parent = nullptr);

protected:
  void initMenus() override;
  void initToolbars() override;
  void configureVisualizationManager() override;
};

}  // namespace rviz2_rc26

#endif  // RVIZ2__RC26__RVIZ2_RC26_VIEWER_FRAME_HPP_
