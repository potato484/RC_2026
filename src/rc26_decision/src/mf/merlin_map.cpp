#include "rc26_decision/mf/merlin_map.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>

namespace {

std::string toLowerCopy(std::string value) {
  std::transform(
      value.begin(), value.end(), value.begin(),
      [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return value;
}

} // namespace

namespace rc26_decision {

MerlinMapManager::MerlinMapManager() {
  depths_.fill(0);
  exit_blocks_ = {10, 11, 12};
  layout_status_ = "uninitialized";
}

void MerlinMapManager::initLegacyRedMapLocked() {
  depths_[1] = 2;
  depths_[2] = 1;
  depths_[3] = 2;
  depths_[4] = 3;
  depths_[5] = 2;
  depths_[6] = 1;
  depths_[7] = 2;
  depths_[8] = 3;
  depths_[9] = 2;
  depths_[10] = 1;
  depths_[11] = 2;
  depths_[12] = 1;
}

void MerlinMapManager::initLegacyBlueMapLocked() {
  depths_[1] = 2;
  depths_[2] = 1;
  depths_[3] = 2;
  depths_[4] = 1;
  depths_[5] = 2;
  depths_[6] = 3;
  depths_[7] = 2;
  depths_[8] = 3;
  depths_[9] = 2;
  depths_[10] = 1;
  depths_[11] = 2;
  depths_[12] = 1;
}

bool MerlinMapManager::initFromWorldLayout(const std::string &team) {
  std::lock_guard<std::mutex> lock(mutex_);

  if (toLowerCopy(team) == "red") {
    initLegacyRedMapLocked();
  } else {
    initLegacyBlueMapLocked();
  }
  exit_blocks_ = {10, 11, 12};
  layout_status_ = "legacy_depth_table";
  return true;
}

bool MerlinMapManager::initRedMap() { return initFromWorldLayout("red"); }

bool MerlinMapManager::initBlueMap() { return initFromWorldLayout("blue"); }

int MerlinMapManager::getDepth(int grid_id) const {
  if (grid_id < 1 || grid_id > 12) {
    return -1;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  return depths_[static_cast<size_t>(grid_id)];
}

bool MerlinMapManager::canTraverse(int from, int to) const {
  const int d1 = getDepth(from);
  const int d2 = getDepth(to);
  if (d1 < 0 || d2 < 0) {
    return false;
  }
  return std::abs(d1 - d2) <= 1;
}

int MerlinMapManager::getAdjacentGrid(int current, MFDirection dir) const {
  if (current < 1 || current > 12) {
    return -1;
  }
  const int row = (current - 1) / 3;
  const int col = (current - 1) % 3;

  switch (dir) {
  case MFDirection::LEFT:
    return (col > 0) ? current - 1 : -1;
  case MFDirection::RIGHT:
    return (col < 2) ? current + 1 : -1;
  case MFDirection::FRONT:
    return (row < 3) ? current + 3 : -1;
  case MFDirection::BACK:
    return (row > 0) ? current - 3 : -1;
  }
  return -1;
}

bool MerlinMapManager::isExitBlock(int grid_id) const {
  std::lock_guard<std::mutex> lock(mutex_);
  return std::find(exit_blocks_.begin(), exit_blocks_.end(), grid_id) !=
         exit_blocks_.end();
}

std::string MerlinMapManager::layoutStatus() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return layout_status_;
}

} // namespace rc26_decision
