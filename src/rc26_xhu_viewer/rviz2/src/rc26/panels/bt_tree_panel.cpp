#include "bt_tree_panel.hpp"

#include <QHeaderView>
#include <QTreeWidget>
#include <QTreeWidgetItem>
#include <QVBoxLayout>

namespace rc26_xhu_viewer
{

BtTreePanel::BtTreePanel(QWidget * parent)
: rviz_common::Panel(parent)
{
  auto * layout = new QVBoxLayout(this);

  tree_widget_ = new QTreeWidget(this);
  tree_widget_->setHeaderHidden(true);
  tree_widget_->setColumnCount(1);

  auto * placeholder = new QTreeWidgetItem(tree_widget_);
  placeholder->setText(0, QStringLiteral("行为树未加载"));
  placeholder->setFlags(placeholder->flags() & ~Qt::ItemIsSelectable);

  layout->addWidget(tree_widget_);
}

void BtTreePanel::load(const rviz_common::Config & config)
{
  rviz_common::Panel::load(config);
}

void BtTreePanel::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);
  config.mapSetValue("Class", "rviz2_rc26/BtTreePanel");
}

}  // namespace rc26_xhu_viewer

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rc26_xhu_viewer::BtTreePanel, rviz_common::Panel)
