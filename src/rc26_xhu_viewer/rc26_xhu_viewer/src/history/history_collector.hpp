#ifndef RC26_XHU_VIEWER__HISTORY__HISTORY_COLLECTOR_HPP_
#define RC26_XHU_VIEWER__HISTORY__HISTORY_COLLECTOR_HPP_

namespace rc26_xhu_viewer
{

class HistoryStore;

class HistoryCollector
{
public:
  explicit HistoryCollector(HistoryStore * store);
  HistoryStore * store() const;

private:
  HistoryStore * store_;
};

}  // namespace rc26_xhu_viewer

#endif  // RC26_XHU_VIEWER__HISTORY__HISTORY_COLLECTOR_HPP_
