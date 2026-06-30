#include "rc26_vision/postprocess/alignment/tip_alignment.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <optional>

namespace rc26_vision {
namespace {

constexpr double kPi = 3.14159265358979323846;

int boxCenterX(const TipTargetCandidate & target)
{
  return target.box.x + target.box.width / 2;
}

int boxArea(const TipTargetCandidate & target)
{
  return target.box.area();
}

std::vector<TipTargetCandidate> collectCandidates(
  const std::vector<Detection> & detections,
  const std::vector<int> & target_class_ids)
{
  std::vector<TipTargetCandidate> candidates;
  candidates.reserve(detections.size());

  for (std::size_t index = 0; index < detections.size(); ++index) {
    const auto & det = detections[index];
    if (!isTipTargetClass(det.class_id, target_class_ids)) {
      continue;
    }

    const int x1 = static_cast<int>(std::floor(det.x1));
    const int y1 = static_cast<int>(std::floor(det.y1));
    const int x2 = static_cast<int>(std::ceil(det.x2));
    const int y2 = static_cast<int>(std::ceil(det.y2));
    const cv::Rect box(x1, y1, std::max(0, x2 - x1), std::max(0, y2 - y1));
    if (box.width <= 0 || box.height <= 0) {
      continue;
    }

    candidates.push_back(TipTargetCandidate{
      box, det.class_id, det.score, static_cast<int>(index)});
  }

  return candidates;
}

std::optional<TipTargetCandidate> chooseClosestToCenter(
  const std::vector<TipTargetCandidate> & candidates,
  int frame_center_x)
{
  std::optional<TipTargetCandidate> best;
  int best_center_distance = -1;
  int best_area = -1;

  for (const auto & candidate : candidates) {
    const int center_distance = std::abs(boxCenterX(candidate) - frame_center_x);
    const int area = boxArea(candidate);
    if (!best.has_value() ||
      center_distance < best_center_distance ||
      (center_distance == best_center_distance && candidate.score > best->score) ||
      (center_distance == best_center_distance && candidate.score == best->score && area > best_area))
    {
      best = candidate;
      best_center_distance = center_distance;
      best_area = area;
    }
  }

  return best;
}

std::optional<TipTargetCandidate> chooseClosestToLockedTarget(
  const std::vector<TipTargetCandidate> & candidates,
  const TipTargetCandidate & locked_target,
  int max_jump_px)
{
  std::optional<TipTargetCandidate> best;
  int best_jump = -1;
  int best_area = -1;
  const int locked_center_x = boxCenterX(locked_target);

  for (const auto & candidate : candidates) {
    const int jump = std::abs(boxCenterX(candidate) - locked_center_x);
    const int area = boxArea(candidate);
    if (jump > max_jump_px) {
      continue;
    }
    if (!best.has_value() ||
      jump < best_jump ||
      (jump == best_jump && candidate.score > best->score) ||
      (jump == best_jump && candidate.score == best->score && area > best_area))
    {
      best = candidate;
      best_jump = jump;
      best_area = area;
    }
  }

  return best;
}

TipTargetSelection makeSelection(
  const TipTargetCandidate & target,
  int frame_center_x,
  const TipTargetLockState & lock_state)
{
  TipTargetSelection selection;
  selection.has_target = true;
  selection.target = target;
  selection.box_cx = boxCenterX(target);
  selection.offset_px = selection.box_cx - frame_center_x;
  selection.locked = lock_state.locked;
  selection.lock_lost_count = lock_state.lost_count;
  return selection;
}

}  // namespace

bool isTipTargetClass(int class_id, const std::vector<int> & target_class_ids)
{
  return std::find(target_class_ids.begin(), target_class_ids.end(), class_id) !=
         target_class_ids.end();
}

TipTargetSelection updateTipAlignmentTarget(
  const std::vector<Detection> & detections,
  int frame_width_px,
  const std::vector<int> & target_class_ids,
  TipTargetLockState & lock_state,
  const TipAlignmentConfig & config)
{
  const int frame_center_x = std::max(0, frame_width_px / 2);
  const int max_jump_px = std::max(0, config.target_lock_max_jump_px);
  const int lost_stop_frames = std::max(1, config.lost_stop_frames);
  const std::vector<TipTargetCandidate> candidates =
    collectCandidates(detections, target_class_ids);

  if (!config.target_lock_enable) {
    lock_state.reset();
    const auto selected = chooseClosestToCenter(candidates, frame_center_x);
    if (!selected.has_value()) {
      return TipTargetSelection{};
    }
    return makeSelection(*selected, frame_center_x, lock_state);
  }

  if (lock_state.locked) {
    const auto tracked =
      chooseClosestToLockedTarget(candidates, lock_state.target, max_jump_px);
    if (tracked.has_value()) {
      lock_state.target = *tracked;
      lock_state.lost_count = 0;
      return makeSelection(lock_state.target, frame_center_x, lock_state);
    }

    ++lock_state.lost_count;
    if (lock_state.lost_count < lost_stop_frames) {
      TipTargetSelection selection;
      selection.locked = true;
      selection.lock_lost_count = lock_state.lost_count;
      return selection;
    }

    lock_state.reset();
  }

  const auto selected = chooseClosestToCenter(candidates, frame_center_x);
  if (!selected.has_value()) {
    return TipTargetSelection{};
  }

  lock_state.locked = true;
  lock_state.target = *selected;
  lock_state.lost_count = 0;
  return makeSelection(lock_state.target, frame_center_x, lock_state);
}

double computeTipAlignmentVy(int offset_px, const TipAlignmentConfig & config)
{
  const int abs_offset = std::abs(offset_px);
  const int tolerance_px = std::max(0, config.tolerance_px);
  double max_speed = std::max(0.0, config.max_speed_mps);
  double min_speed = std::max(0.0, config.min_speed_mps);
  if (min_speed > max_speed) {
    min_speed = max_speed;
  }

  if (abs_offset <= tolerance_px || max_speed <= 0.0) {
    return 0.0;
  }

  double speed = static_cast<double>(abs_offset) * std::max(0.0, config.kp);
  speed = std::clamp(speed, min_speed, max_speed);

  double direction = offset_px > 0 ? -1.0 : 1.0;
  if (config.invert_direction) {
    direction = -direction;
  }
  return direction * speed;
}

double normalizeTipYawError(double yaw_error_rad)
{
  if (!std::isfinite(yaw_error_rad)) {
    return 0.0;
  }
  while (yaw_error_rad > kPi) {
    yaw_error_rad -= 2.0 * kPi;
  }
  while (yaw_error_rad < -kPi) {
    yaw_error_rad += 2.0 * kPi;
  }
  return yaw_error_rad;
}

TipHeadingControl computeTipHeadingControl(
  double current_yaw_rad,
  const TipAlignmentConfig & config)
{
  TipHeadingControl control;
  if (!config.heading_hold_enable) {
    return control;
  }

  if (!std::isfinite(current_yaw_rad) || !std::isfinite(config.target_yaw_rad)) {
    control.aligned = false;
    control.within_gate = false;
    control.allow_lateral = false;
    return control;
  }

  const double tolerance = std::max(0.0, config.heading_tolerance_rad);
  const double gate = std::max(tolerance, config.heading_gate_rad);
  const double max_speed = std::max(0.0, config.heading_max_speed_radps);
  const double kp = std::max(0.0, config.heading_kp);

  control.yaw_error_rad = normalizeTipYawError(config.target_yaw_rad - current_yaw_rad);
  const double abs_error = std::abs(control.yaw_error_rad);
  control.aligned = abs_error <= tolerance;
  control.within_gate = abs_error <= gate;
  control.allow_lateral = control.within_gate;

  if (control.aligned || max_speed <= 0.0 || kp <= 0.0) {
    control.angular_z_radps = 0.0;
    return control;
  }

  const double speed = std::clamp(abs_error * kp, 0.0, max_speed);
  control.angular_z_radps = (control.yaw_error_rad >= 0.0 ? 1.0 : -1.0) * speed;
  return control;
}

double computeTipApproachVx(double speed_mps)
{
  return speed_mps == 0.0 ? 0.0 : -std::abs(speed_mps);
}

}  // namespace rc26_vision
