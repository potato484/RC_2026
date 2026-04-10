#include "serial_panel.hpp"

#include <QLabel>
#include <QTextEdit>
#include <QVBoxLayout>

namespace rc26_xhu_viewer
{

SerialPanel::SerialPanel(QWidget * parent)
: rviz_common::Panel(parent)
{
  auto * layout = new QVBoxLayout(this);

  title_label_ = new QLabel(QStringLiteral("原始帧日志"));
  title_label_->setStyleSheet("font-weight: bold;");
  layout->addWidget(title_label_);

  frame_log_ = new QTextEdit();
  frame_log_->setReadOnly(true);
  frame_log_->setPlaceholderText(QStringLiteral("等待数据..."));
  layout->addWidget(frame_log_);
}

void SerialPanel::load(const rviz_common::Config & config)
{
  rviz_common::Panel::load(config);
}

void SerialPanel::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);
  config.mapSetValue("Class", "rviz2_rc26/SerialPanel");
}

}  // namespace rc26_xhu_viewer

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rc26_xhu_viewer::SerialPanel, rviz_common::Panel)
