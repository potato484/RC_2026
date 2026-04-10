#ifndef RC26_XHU_VIEWER__PANELS__BT_TREE_PANEL_HPP_
#define RC26_XHU_VIEWER__PANELS__BT_TREE_PANEL_HPP_

#include <rviz_common/panel.hpp>

class QTreeWidget;

namespace rc26_xhu_viewer
{

class BtTreePanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit BtTreePanel(QWidget * parent = nullptr);

  void load(const rviz_common::Config & config) override;
  void save(rviz_common::Config config) const override;

private:
  QTreeWidget * tree_widget_;
};

}  // namespace rc26_xhu_viewer

#endif  // RC26_XHU_VIEWER__PANELS__BT_TREE_PANEL_HPP_
