#ifndef RC26_XHU_VIEWER__RC26_XHU_VIEWER_FRAME_HPP_
#define RC26_XHU_VIEWER__RC26_XHU_VIEWER_FRAME_HPP_

#include "rviz_common/visualization_frame.hpp"

namespace rc26_xhu_viewer
{

class RC26XhuViewerFrame : public rviz_common::VisualizationFrame
{
  Q_OBJECT

public:
  explicit RC26XhuViewerFrame(
    rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr rviz_ros_node,
    QWidget * parent = nullptr);

protected:
  void initMenus() override;
  void initToolbars() override;
  void configureVisualizationManager() override;
};

}  // namespace rc26_xhu_viewer

#endif  // RC26_XHU_VIEWER__RC26_XHU_VIEWER_FRAME_HPP_
