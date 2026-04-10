#ifndef RC26_XHU_VIEWER__HISTORY__HISTORY_STORE_HPP_
#define RC26_XHU_VIEWER__HISTORY__HISTORY_STORE_HPP_

#include <string>
#include <vector>
#include <mutex>
#include <atomic>
#include <thread>
#include <cstdint>

struct sqlite3;  // forward declare

namespace rc26_xhu_viewer
{

struct HistoryEvent
{
  int64_t timestamp_ms;
  std::string source;
  std::string level;    // INFO, WARN, ERROR
  std::string message;
};

struct ScalarTrend
{
  int64_t timestamp_ms;
  std::string name;
  double value;
};

class HistoryStore
{
public:
  HistoryStore();
  ~HistoryStore();

  bool open(const std::string & path = "/tmp/rc26_xhu_viewer_history.db");
  void close();

  void insertEvent(const HistoryEvent & event);
  void insertScalar(const ScalarTrend & trend);
  void insertStateSnapshot(
    int64_t timestamp_ms, const std::string & key, const std::string & json_value);

  std::vector<HistoryEvent> queryEvents(int64_t from_ms, int64_t to_ms, int limit = 1000);
  std::vector<ScalarTrend> queryScalars(
    const std::string & name, int64_t from_ms, int64_t to_ms, int limit = 5000);

private:
  void createTables();
  void cleanupLoop();
  void purgeOlderThan(int64_t threshold_ms);

  sqlite3 * db_{nullptr};
  std::mutex mutex_;
  std::atomic<bool> running_{false};
  std::thread cleanup_thread_;
};

}  // namespace rc26_xhu_viewer

#endif  // RC26_XHU_VIEWER__HISTORY__HISTORY_STORE_HPP_
