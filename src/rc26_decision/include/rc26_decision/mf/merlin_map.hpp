#pragma once

#include <array>
#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

namespace rc26_decision {

enum class MFDirection : uint8_t { LEFT = 0, FRONT = 1, RIGHT = 2, BACK = 3 };

class MerlinMapManager {
public:
  MerlinMapManager();
  bool initRedMap();
  bool initBlueMap();
  int getDepth(int grid_id) const;
  bool canTraverse(int from, int to) const;
  int getAdjacentGrid(int current, MFDirection dir) const;
  bool isExitBlock(int grid_id) const;
  std::string layoutStatus() const;

private:
  bool initFromWorldLayout(const std::string &team);
  void initLegacyRedMapLocked();
  void initLegacyBlueMapLocked();

  mutable std::mutex mutex_;
  std::array<int, 13> depths_; // index 0 unused, 1-12 for grids
  std::vector<int> exit_blocks_;
  std::string layout_status_;
};

} // namespace rc26_decision
