#include "history_collector.hpp"

namespace rc26_xhu_viewer
{

HistoryCollector::HistoryCollector(HistoryStore * store)
: store_(store)
{
}

HistoryStore * HistoryCollector::store() const
{
  return store_;
}

}  // namespace rc26_xhu_viewer
