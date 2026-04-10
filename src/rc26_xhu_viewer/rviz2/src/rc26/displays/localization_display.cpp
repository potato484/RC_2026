#include "localization_display.hpp"

#include <rviz_common/display_context.hpp>

namespace rc26_xhu_viewer
{

LocalizationDisplay::LocalizationDisplay()
{
  pose_status_ = new rviz_common::properties::StringProperty(
    "Pose", QString::fromUtf8("等待数据..."),
    QString::fromUtf8("PoseWithCovarianceStamped 接收状态"), this);

  keyframe_status_ = new rviz_common::properties::StringProperty(
    "Keyframes", QString::fromUtf8("等待数据..."),
    QString::fromUtf8("关键帧数组接收状态"), this);

  loop_closure_status_ = new rviz_common::properties::StringProperty(
    "LoopClosures", QString::fromUtf8("等待数据..."),
    QString::fromUtf8("回环检测接收状态"), this);
}

void LocalizationDisplay::onInitialize()
{
  Display::onInitialize();
}

void LocalizationDisplay::update(float /*wall_dt*/, float /*ros_dt*/)
{
  // future: check subscription freshness and update status text
}

void LocalizationDisplay::reset()
{
  Display::reset();
  pose_status_->setValue(QString::fromUtf8("等待数据..."));
  keyframe_status_->setValue(QString::fromUtf8("等待数据..."));
  loop_closure_status_->setValue(QString::fromUtf8("等待数据..."));
}

}  // namespace rc26_xhu_viewer

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rc26_xhu_viewer::LocalizationDisplay, rviz_common::Display)
