#include "dynamic_prediction_display.hpp"

#include <rviz_common/display_context.hpp>

namespace rc26_xhu_viewer
{

DynamicPredictionDisplay::DynamicPredictionDisplay()
{
  prediction_status_ = new rviz_common::properties::StringProperty(
    "Predictions", QString::fromUtf8("等待数据..."),
    QString::fromUtf8("动态障碍物预测数组接收状态"), this);
}

void DynamicPredictionDisplay::onInitialize()
{
  Display::onInitialize();
}

void DynamicPredictionDisplay::update(float /*wall_dt*/, float /*ros_dt*/)
{
  // future: check subscription freshness and update status text
}

void DynamicPredictionDisplay::reset()
{
  Display::reset();
  prediction_status_->setValue(QString::fromUtf8("等待数据..."));
}

}  // namespace rc26_xhu_viewer

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  rc26_xhu_viewer::DynamicPredictionDisplay, rviz_common::Display)
