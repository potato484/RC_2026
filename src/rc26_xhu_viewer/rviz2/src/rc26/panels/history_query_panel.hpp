#ifndef RC26_XHU_VIEWER__PANELS__HISTORY_QUERY_PANEL_HPP_
#define RC26_XHU_VIEWER__PANELS__HISTORY_QUERY_PANEL_HPP_

#include <rviz_common/panel.hpp>

class QComboBox;
class QLineEdit;
class QPushButton;
class QTextEdit;

namespace rc26_xhu_viewer
{

class HistoryStore;

class HistoryQueryPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit HistoryQueryPanel(QWidget * parent = nullptr);

  void setHistoryStore(HistoryStore * store);

  void load(const rviz_common::Config & config) override;
  void save(rviz_common::Config config) const override;

private Q_SLOTS:
  void onQueryClicked();

private:
  HistoryStore * store_{nullptr};
  QLineEdit * search_edit_{nullptr};
  QComboBox * range_combo_{nullptr};
  QPushButton * query_btn_{nullptr};
  QTextEdit * result_display_{nullptr};
};

}  // namespace rc26_xhu_viewer

#endif  // RC26_XHU_VIEWER__PANELS__HISTORY_QUERY_PANEL_HPP_
