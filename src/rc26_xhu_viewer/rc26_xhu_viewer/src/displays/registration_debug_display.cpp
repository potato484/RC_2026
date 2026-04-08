#include "registration_debug_display.hpp"

#include <rviz_common/display_context.hpp>

namespace rc26_xhu_viewer
{

RegistrationDebugDisplay::RegistrationDebugDisplay()
{
  fitness_status_ = new rviz_common::properties::StringProperty(
    "FitnessScore", QString::fromUtf8("等待数据..."),
    QString::fromUtf8("配准适配度分数"), this);

  inlier_status_ = new rviz_common::properties::StringProperty(
    "InlierCount", QString::fromUtf8("等待数据..."),
    QString::fromUtf8("内点数量"), this);

  converged_status_ = new rviz_common::properties::StringProperty(
    "Converged", QString::fromUtf8("等待数据..."),
    QString::fromUtf8("配准是否收敛"), this);
}

void RegistrationDebugDisplay::onInitialize()
{
  Display::onInitialize();
}

void RegistrationDebugDisplay::update(float /*wall_dt*/, float /*ros_dt*/)
{
  // future: check subscription freshness and update status text
}

void RegistrationDebugDisplay::reset()
{
  Display::reset();
  fitness_status_->setValue(QString::fromUtf8("等待数据..."));
  inlier_status_->setValue(QString::fromUtf8("等待数据..."));
  converged_status_->setValue(QString::fromUtf8("等待数据..."));
}

}  // namespace rc26_xhu_viewer

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  rc26_xhu_viewer::RegistrationDebugDisplay, rviz_common::Display)
