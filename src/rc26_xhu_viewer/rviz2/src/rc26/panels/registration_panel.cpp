#include "registration_panel.hpp"

#include <QLabel>
#include <QVBoxLayout>

namespace rc26_xhu_viewer
{

RegistrationPanel::RegistrationPanel(QWidget * parent)
: rviz_common::Panel(parent)
{
  auto * layout = new QVBoxLayout(this);

  fitness_score_label_ = new QLabel(QStringLiteral("适配度得分: 等待数据..."));
  inlier_count_label_ = new QLabel(QStringLiteral("内点数: 等待数据..."));
  convergence_label_ = new QLabel(QStringLiteral("收敛状态: 等待数据..."));

  layout->addWidget(fitness_score_label_);
  layout->addWidget(inlier_count_label_);
  layout->addWidget(convergence_label_);
}

void RegistrationPanel::load(const rviz_common::Config & config)
{
  rviz_common::Panel::load(config);
}

void RegistrationPanel::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);
  config.mapSetValue("Class", "rviz2_rc26/RegistrationPanel");
}

}  // namespace rc26_xhu_viewer

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rc26_xhu_viewer::RegistrationPanel, rviz_common::Panel)
