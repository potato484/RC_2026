#pragma once

#include <optional>
#include <vector>

#include <opencv2/core.hpp>

#include "rc26_vision/shared/contracts/vision_types.hpp"

namespace rc26_vision {

struct TipAlignmentConfig
{
  bool target_lock_enable{true};
  int target_lock_max_jump_px{160};
  int lost_stop_frames{3};
  int tolerance_px{20};
  double kp{0.0015};
  double min_speed_mps{0.04};
  double max_speed_mps{0.15};
  bool invert_direction{false};
};

struct TipTargetCandidate
{
  cv::Rect box{};
  int class_id{-1};
  float score{0.0F};
};

struct TipTargetLockState
{
  bool locked{false};
  TipTargetCandidate target{};
  int lost_count{0};

  void reset()
  {
    locked = false;
    target = TipTargetCandidate{};
    lost_count = 0;
  }
};

struct TipTargetSelection
{
  bool has_target{false};
  TipTargetCandidate target{};
  int box_cx{-1};
  int offset_px{0};
  bool locked{false};
  int lock_lost_count{0};
};

bool isTipTargetClass(int class_id, const std::vector<int> & target_class_ids);

TipTargetSelection updateTipAlignmentTarget(
  const std::vector<Detection> & detections,
  int frame_width_px,
  const std::vector<int> & target_class_ids,
  TipTargetLockState & lock_state,
  const TipAlignmentConfig & config);

double computeTipAlignmentVy(int offset_px, const TipAlignmentConfig & config);

double computeTipApproachVx(double speed_mps);

}  // namespace rc26_vision
