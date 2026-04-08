#include "history_store.hpp"

#include <sqlite3.h>

#include <chrono>
#include <cstring>

namespace rc26_xhu_viewer
{

namespace
{

int64_t nowMs()
{
  using namespace std::chrono;
  return duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
}

constexpr int64_t kRetentionMs = 2 * 60 * 60 * 1000;  // 2 hours
constexpr int kCleanupIntervalSec = 30;

}  // namespace

HistoryStore::HistoryStore() = default;

HistoryStore::~HistoryStore()
{
  close();
}

bool HistoryStore::open(const std::string & path)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (db_) {
    return true;
  }
  int rc = sqlite3_open(path.c_str(), &db_);
  if (rc != SQLITE_OK) {
    db_ = nullptr;
    return false;
  }
  // WAL mode for better concurrent read/write
  sqlite3_exec(db_, "PRAGMA journal_mode=WAL;", nullptr, nullptr, nullptr);
  sqlite3_exec(db_, "PRAGMA synchronous=NORMAL;", nullptr, nullptr, nullptr);

  createTables();

  running_ = true;
  cleanup_thread_ = std::thread(&HistoryStore::cleanupLoop, this);
  return true;
}

void HistoryStore::close()
{
  running_ = false;
  if (cleanup_thread_.joinable()) {
    cleanup_thread_.join();
  }
  std::lock_guard<std::mutex> lock(mutex_);
  if (db_) {
    sqlite3_close(db_);
    db_ = nullptr;
  }
}

void HistoryStore::createTables()
{
  const char * sqls[] = {
    "CREATE TABLE IF NOT EXISTS events ("
    "  id INTEGER PRIMARY KEY AUTOINCREMENT,"
    "  timestamp_ms INTEGER NOT NULL,"
    "  source TEXT NOT NULL,"
    "  level TEXT NOT NULL,"
    "  message TEXT NOT NULL"
    ");",

    "CREATE TABLE IF NOT EXISTS state_snapshots ("
    "  id INTEGER PRIMARY KEY AUTOINCREMENT,"
    "  timestamp_ms INTEGER NOT NULL,"
    "  key TEXT NOT NULL,"
    "  json_value TEXT NOT NULL"
    ");",

    "CREATE TABLE IF NOT EXISTS scalar_trends ("
    "  id INTEGER PRIMARY KEY AUTOINCREMENT,"
    "  timestamp_ms INTEGER NOT NULL,"
    "  name TEXT NOT NULL,"
    "  value REAL NOT NULL"
    ");",

    "CREATE INDEX IF NOT EXISTS idx_events_ts ON events(timestamp_ms);",
    "CREATE INDEX IF NOT EXISTS idx_state_snapshots_ts ON state_snapshots(timestamp_ms);",
    "CREATE INDEX IF NOT EXISTS idx_scalar_trends_ts ON scalar_trends(timestamp_ms);",
  };

  for (auto * sql : sqls) {
    sqlite3_exec(db_, sql, nullptr, nullptr, nullptr);
  }
}

void HistoryStore::insertEvent(const HistoryEvent & event)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!db_) { return; }

  sqlite3_stmt * stmt = nullptr;
  const char * sql =
    "INSERT INTO events (timestamp_ms, source, level, message) VALUES (?, ?, ?, ?);";
  if (sqlite3_prepare_v2(db_, sql, -1, &stmt, nullptr) != SQLITE_OK) { return; }

  sqlite3_bind_int64(stmt, 1, event.timestamp_ms);
  sqlite3_bind_text(stmt, 2, event.source.c_str(), -1, SQLITE_TRANSIENT);
  sqlite3_bind_text(stmt, 3, event.level.c_str(), -1, SQLITE_TRANSIENT);
  sqlite3_bind_text(stmt, 4, event.message.c_str(), -1, SQLITE_TRANSIENT);
  sqlite3_step(stmt);
  sqlite3_finalize(stmt);
}

void HistoryStore::insertScalar(const ScalarTrend & trend)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!db_) { return; }

  sqlite3_stmt * stmt = nullptr;
  const char * sql =
    "INSERT INTO scalar_trends (timestamp_ms, name, value) VALUES (?, ?, ?);";
  if (sqlite3_prepare_v2(db_, sql, -1, &stmt, nullptr) != SQLITE_OK) { return; }

  sqlite3_bind_int64(stmt, 1, trend.timestamp_ms);
  sqlite3_bind_text(stmt, 2, trend.name.c_str(), -1, SQLITE_TRANSIENT);
  sqlite3_bind_double(stmt, 3, trend.value);
  sqlite3_step(stmt);
  sqlite3_finalize(stmt);
}

void HistoryStore::insertStateSnapshot(
  int64_t timestamp_ms, const std::string & key, const std::string & json_value)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!db_) { return; }

  sqlite3_stmt * stmt = nullptr;
  const char * sql =
    "INSERT INTO state_snapshots (timestamp_ms, key, json_value) VALUES (?, ?, ?);";
  if (sqlite3_prepare_v2(db_, sql, -1, &stmt, nullptr) != SQLITE_OK) { return; }

  sqlite3_bind_int64(stmt, 1, timestamp_ms);
  sqlite3_bind_text(stmt, 2, key.c_str(), -1, SQLITE_TRANSIENT);
  sqlite3_bind_text(stmt, 3, json_value.c_str(), -1, SQLITE_TRANSIENT);
  sqlite3_step(stmt);
  sqlite3_finalize(stmt);
}

std::vector<HistoryEvent> HistoryStore::queryEvents(
  int64_t from_ms, int64_t to_ms, int limit)
{
  std::vector<HistoryEvent> results;
  std::lock_guard<std::mutex> lock(mutex_);
  if (!db_) { return results; }

  sqlite3_stmt * stmt = nullptr;
  const char * sql =
    "SELECT timestamp_ms, source, level, message FROM events "
    "WHERE timestamp_ms BETWEEN ? AND ? ORDER BY timestamp_ms LIMIT ?;";
  if (sqlite3_prepare_v2(db_, sql, -1, &stmt, nullptr) != SQLITE_OK) { return results; }

  sqlite3_bind_int64(stmt, 1, from_ms);
  sqlite3_bind_int64(stmt, 2, to_ms);
  sqlite3_bind_int(stmt, 3, limit);

  while (sqlite3_step(stmt) == SQLITE_ROW) {
    HistoryEvent ev;
    ev.timestamp_ms = sqlite3_column_int64(stmt, 0);
    ev.source = reinterpret_cast<const char *>(sqlite3_column_text(stmt, 1));
    ev.level = reinterpret_cast<const char *>(sqlite3_column_text(stmt, 2));
    ev.message = reinterpret_cast<const char *>(sqlite3_column_text(stmt, 3));
    results.push_back(std::move(ev));
  }
  sqlite3_finalize(stmt);
  return results;
}

std::vector<ScalarTrend> HistoryStore::queryScalars(
  const std::string & name, int64_t from_ms, int64_t to_ms, int limit)
{
  std::vector<ScalarTrend> results;
  std::lock_guard<std::mutex> lock(mutex_);
  if (!db_) { return results; }

  sqlite3_stmt * stmt = nullptr;
  const char * sql =
    "SELECT timestamp_ms, name, value FROM scalar_trends "
    "WHERE name = ? AND timestamp_ms BETWEEN ? AND ? ORDER BY timestamp_ms LIMIT ?;";
  if (sqlite3_prepare_v2(db_, sql, -1, &stmt, nullptr) != SQLITE_OK) { return results; }

  sqlite3_bind_text(stmt, 1, name.c_str(), -1, SQLITE_TRANSIENT);
  sqlite3_bind_int64(stmt, 2, from_ms);
  sqlite3_bind_int64(stmt, 3, to_ms);
  sqlite3_bind_int(stmt, 4, limit);

  while (sqlite3_step(stmt) == SQLITE_ROW) {
    ScalarTrend t;
    t.timestamp_ms = sqlite3_column_int64(stmt, 0);
    t.name = reinterpret_cast<const char *>(sqlite3_column_text(stmt, 1));
    t.value = sqlite3_column_double(stmt, 2);
    results.push_back(std::move(t));
  }
  sqlite3_finalize(stmt);
  return results;
}

void HistoryStore::cleanupLoop()
{
  while (running_) {
    for (int i = 0; i < kCleanupIntervalSec && running_; ++i) {
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    if (!running_) { break; }
    purgeOlderThan(nowMs() - kRetentionMs);
  }
}

void HistoryStore::purgeOlderThan(int64_t threshold_ms)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!db_) { return; }

  const char * tables[] = {"events", "state_snapshots", "scalar_trends"};
  for (auto * table : tables) {
    std::string sql = "DELETE FROM ";
    sql += table;
    sql += " WHERE timestamp_ms < ?;";

    sqlite3_stmt * stmt = nullptr;
    if (sqlite3_prepare_v2(db_, sql.c_str(), -1, &stmt, nullptr) == SQLITE_OK) {
      sqlite3_bind_int64(stmt, 1, threshold_ms);
      sqlite3_step(stmt);
    }
    sqlite3_finalize(stmt);
  }
}

}  // namespace rc26_xhu_viewer
