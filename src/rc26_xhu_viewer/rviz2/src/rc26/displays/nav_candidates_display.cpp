#include "nav_candidates_display.hpp"

#include <rviz_common/display_context.hpp>

namespace rc26_xhu_viewer
{

NavCandidatesDisplay::NavCandidatesDisplay()
{
  candidates_status_ = new rviz_common::properties::StringProperty(
    "Candidates", QString::fromUtf8("等待数据..."),
    QString::fromUtf8("局部候选轨迹 MarkerArray 接收状态"), this);

  trajectory_status_ = new rviz_common::properties::StringProperty(
    "SelectedPath", QString::fromUtf8("等待数据..."),
    QString::fromUtf8("选中轨迹 Path 接收状态"), this);
}

void NavCandidatesDisplay::onInitialize()
{
  Display::onInitialize();
}

void NavCandidatesDisplay::update(float /*wall_dt*/, float /*ros_dt*/)
{
  // future: check subscription freshness and update status text
}

void NavCandidatesDisplay::reset()
{
  Display::reset();
  candidates_status_->setValue(QString::fromUtf8("等待数据..."));
  trajectory_status_->setValue(QString::fromUtf8("等待数据..."));
}

}  // namespace rc26_xhu_viewer

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rc26_xhu_viewer::NavCandidatesDisplay, rviz_common::Display)
