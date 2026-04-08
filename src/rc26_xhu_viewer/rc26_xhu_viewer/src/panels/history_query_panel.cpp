#include "history_query_panel.hpp"

#include <QComboBox>
#include <QHBoxLayout>
#include <QLineEdit>
#include <QPushButton>
#include <QTextEdit>
#include <QVBoxLayout>

#include <chrono>

#include "../history/history_store.hpp"

namespace rc26_xhu_viewer
{

namespace
{

int64_t nowMs()
{
  using namespace std::chrono;
  return duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
}

}  // namespace

HistoryQueryPanel::HistoryQueryPanel(QWidget * parent)
: rviz_common::Panel(parent)
{
  auto * layout = new QVBoxLayout(this);

  auto * top_layout = new QHBoxLayout();
  search_edit_ = new QLineEdit();
  search_edit_->setPlaceholderText(QStringLiteral("搜索关键字..."));
  top_layout->addWidget(search_edit_);

  range_combo_ = new QComboBox();
  range_combo_->addItem(QStringLiteral("最近 5 分钟"));
  range_combo_->addItem(QStringLiteral("最近 30 分钟"));
  range_combo_->addItem(QStringLiteral("最近 1 小时"));
  range_combo_->addItem(QStringLiteral("全部"));
  top_layout->addWidget(range_combo_);

  query_btn_ = new QPushButton(QStringLiteral("查询"));
  top_layout->addWidget(query_btn_);
  layout->addLayout(top_layout);

  result_display_ = new QTextEdit();
  result_display_->setReadOnly(true);
  result_display_->setPlainText(QStringLiteral("历史库未连接"));
  layout->addWidget(result_display_);

  connect(query_btn_, &QPushButton::clicked, this, &HistoryQueryPanel::onQueryClicked);
}

void HistoryQueryPanel::setHistoryStore(HistoryStore * store)
{
  store_ = store;
  if (store_) {
    result_display_->setPlainText(QStringLiteral("历史库已连接，请输入查询条件。"));
  } else {
    result_display_->setPlainText(QStringLiteral("历史库未连接"));
  }
}

void HistoryQueryPanel::onQueryClicked()
{
  if (!store_) {
    result_display_->setPlainText(QStringLiteral("历史库未连接"));
    return;
  }

  int64_t now = nowMs();
  int64_t from_ms = 0;
  int idx = range_combo_->currentIndex();
  switch (idx) {
    case 0: from_ms = now - 5 * 60 * 1000; break;       // 5 min
    case 1: from_ms = now - 30 * 60 * 1000; break;      // 30 min
    case 2: from_ms = now - 60 * 60 * 1000; break;      // 1 hour
    default: from_ms = 0; break;                          // all
  }

  auto events = store_->queryEvents(from_ms, now);
  QString keyword = search_edit_->text().trimmed();

  QString output;
  int count = 0;
  for (const auto & ev : events) {
    QString msg = QString::fromStdString(ev.message);
    QString src = QString::fromStdString(ev.source);
    if (!keyword.isEmpty() && !msg.contains(keyword, Qt::CaseInsensitive) &&
        !src.contains(keyword, Qt::CaseInsensitive))
    {
      continue;
    }
    output += QStringLiteral("[%1] [%2] %3: %4\n")
      .arg(ev.timestamp_ms)
      .arg(QString::fromStdString(ev.level))
      .arg(src)
      .arg(msg);
    ++count;
  }

  if (count == 0) {
    result_display_->setPlainText(QStringLiteral("无匹配记录。"));
  } else {
    result_display_->setPlainText(
      QStringLiteral("共 %1 条记录:\n\n").arg(count) + output);
  }
}

void HistoryQueryPanel::load(const rviz_common::Config & config)
{
  rviz_common::Panel::load(config);
}

void HistoryQueryPanel::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);
  config.mapSetValue("Class", "rc26_xhu_viewer/HistoryQueryPanel");
}

}  // namespace rc26_xhu_viewer

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rc26_xhu_viewer::HistoryQueryPanel, rviz_common::Panel)
