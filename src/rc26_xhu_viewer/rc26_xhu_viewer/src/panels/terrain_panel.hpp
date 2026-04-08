#ifndef RC26_XHU_VIEWER__PANELS__TERRAIN_PANEL_HPP_
#define RC26_XHU_VIEWER__PANELS__TERRAIN_PANEL_HPP_

#include <rviz_common/panel.hpp>

class QLabel;

namespace rc26_xhu_viewer
{

class TerrainPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit TerrainPanel(QWidget * parent = nullptr);

  void load(const rviz_common::Config & config) override;
  void save(rviz_common::Config config) const override;

private:
  QLabel * semantic_layer_label_;
  QLabel * terrain_features_label_;
};

}  // namespace rc26_xhu_viewer

#endif  // RC26_XHU_VIEWER__PANELS__TERRAIN_PANEL_HPP_
