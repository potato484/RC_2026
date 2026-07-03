#include "rc26_decision/mf_preselection/mf_preselection_flow.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <utility>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include "rc26_decision/decision_failure.hpp"
#include "rc26_decision/mf/merlin_map.hpp"
#include "rc26_decision/team_color.hpp"
#include "rc26_serial/protocol.hpp"
#include "rc26_vision/inference/config/model_profile_loader.hpp"
#include "rc26_vision/postprocess/alignment/tip_alignment.hpp"
#include "rc26_vision/shared/sensors/depth_roi_sampler.hpp"
#include "rc26_vision/shared/target/visual_target_match.hpp"

namespace rc26_decision {

namespace {

// 本文件实现的是“梅林区预选赛”专属的 BT 动作节点。
//
// 外层 BehaviorTree 只看到一个 StatefulActionNode：启动后持续返回 RUNNING，
// 完成整条梅林预选赛流程后返回 SUCCESS，任何关键资源缺失、机构拒绝、
// 台阶事件超时或运动超时都会返回 FAILURE。真实的比赛流程由 phase_ 和
// stair_phase_ 两套内部状态机推进，避免把几十个细粒度阶段全部暴露到 XML。
//
// 运行边界：
// - 决策层只编排策略和调用下层契约，不解析串口帧；
// - 视觉目标来自 rc26_vision 的帧快照；
// - 机构动作通过 /mechanism/send_command service 和反馈 topic 完成；
  // - 运动直接发布 cmd_vel，因此运行本树前必须停用遥控等其它
//   cmd_vel 权威。
//
// 主路线简表：
// 1. 在 2 号入口检测 R2 KFS；未发现则高抬升机械臂并横移探测 1/3 号入口；
// 2. 回到中间入口后上台阶进入 grid2；
// 3. 沿中间列 grid2 -> grid5 -> grid8 -> grid11 推进，按行执行前方守卫检测、
//    左侧/背向扫描、R1 等待、假 KFS 避障和 R2 KFS 夹取；
// 4. 第四行强制转向收尾，必要时遇假 KFS 180 度转向，否则经 grid12 对齐后下台阶离场；
// 5. 离场后返回 SUCCESS，外层树通常接 WaitForever 保持静止。

constexpr double kPi = 3.14159265358979323846;
constexpr double kDeg2Rad = kPi / 180.0;
constexpr double kMinTimeoutS = 0.001;
constexpr double kMinSpeed = 0.001;
constexpr double kDefaultKfsOdomXyKp = 0.8;
constexpr double kDefaultKfsApproachOdomToleranceM = 0.02;
constexpr double kDefaultKfsOdomYawToleranceDeg = 3.0;
constexpr double kDefaultKfsApproachMinSpeedMps = 0.03;
constexpr int kFinalExitVirtualGrid = 0;

std::string seqText(int seq) {
  return seq >= 0 ? std::to_string(seq) : std::string("未知");
}

const char *yesNoText(bool value) { return value ? "是" : "否"; }

std::string metersText(bool has_value, double value_m) {
  if (!has_value || !std::isfinite(value_m) || value_m <= 0.0) {
    return "无";
  }
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(3) << value_m << "m";
  return oss.str();
}

std::string matTypeName(int type) {
  switch (type) {
  case CV_8UC1:
    return "CV_8UC1";
  case CV_16UC1:
    return "CV_16UC1";
  case CV_32FC1:
    return "CV_32FC1";
  default:
    break;
  }
  std::ostringstream oss;
  oss << "type_" << type;
  return oss.str();
}

std::string matTypeName(const cv::Mat &mat) {
  return mat.empty() ? std::string("空") : matTypeName(mat.type());
}

std::string byteHex(uint8_t value) {
  std::ostringstream oss;
  oss << std::uppercase << std::hex << std::setw(2) << std::setfill('0')
      << static_cast<unsigned int>(value);
  return oss.str();
}

bool startsWith(const std::string &text, const std::string &prefix) {
  return text.rfind(prefix, 0) == 0;
}

std::string suffixAfter(const std::string &text, const std::string &prefix) {
  return startsWith(text, prefix) ? text.substr(prefix.size()) : std::string{};
}

std::string translateMfFailureReason(const std::string &reason) {
  if (reason.empty()) {
    return "未知原因";
  }
  if (reason == "vision_start_failed") {
    return "KFS 视觉启动失败";
  }
  if (reason == "invalid_fake_avoid_target_grid") {
    return "假 KFS 避障目标格非法";
  }
  if (reason == "invalid_fake_avoid_transition") {
    return "假 KFS 避障格间转换非法";
  }
  if (reason == "invalid_fake_avoid_forward_target") {
    return "假 KFS 旁列前向推进目标格非法";
  }
  if (reason == "no_transition_target") {
    return "当前格没有合法的下一格转换目标";
  }
  if (reason == "entry_return_to_center_reference_missing") {
    return "入口回中线缺少横移参考";
  }
  if (reason == "entry_return_to_center_invalid_offset") {
    return "入口回中线横移偏差非法";
  }
  if (reason == "entry_resume_probe_reference_missing") {
    return "入口恢复探测横移缺少参考";
  }
  if (reason == "move_runtime_missing") {
    return "相对移动运行上下文缺失";
  }
  if (reason == "turn_runtime_missing") {
    return "转向运行上下文缺失";
  }
  if (reason == "center_runtime_missing") {
    return "格中心归位运行上下文缺失";
  }
  if (reason == "entry_center_target_prepare_failed") {
    return "入口格中心目标计算失败";
  }
  if (reason == "invalid_transition") {
    return "格间转换参数非法";
  }
  if (reason == "entry_center_start_failed") {
    return "入口格中心前进启动失败";
  }
  if (reason == "transition_center_start_failed") {
    return "格间转换后的格中心归位启动失败";
  }
  if (reason == "fake_avoid_center_start_failed") {
    return "假 KFS 避障后的格中心归位启动失败";
  }
  if (reason == "final_exit_center_start_failed") {
    return "最终离场格中心归位启动失败";
  }
  if (reason == "post_grab_center_start_failed") {
    return "夹取后格中心归位启动失败";
  }
  if (reason == "wheel_event_timeout") {
    return "等待激光高度突变事件超时";
  }
  if (reason == "kfs_visual_align_state_missing") {
    return "KFS 视觉横移对齐状态缺失";
  }
  if (reason == "kfs_visual_align_target_missing") {
    return "KFS 视觉横移对齐目标缺失";
  }
  if (reason == "kfs_odom_approach_state_missing") {
    return "KFS odom 前向趋近状态缺失";
  }
  if (reason == "kfs_odom_approach_depth_missing") {
    return "KFS odom 前向趋近缺少有效深度";
  }
  if (reason == "kfs_odom_approach_speed_non_positive") {
    return "KFS odom 前向趋近速度配置非正";
  }
  if (reason == "kfs_odom_approach_plan_timeout") {
    return "KFS odom 前向趋近规划时长超过安全超时";
  }
  if (reason == "kfs_odom_approach_target_missing") {
    return "KFS odom 前向趋近目标缺失";
  }
  if (reason == "kfs_odom_approach_runtime_failed") {
    return "KFS odom 前向趋近运行失败";
  }
  if (reason == "grab_kfs_failed") {
    return "KFS 夹取命令失败";
  }
  if (reason == "grab_verify_target_missing") {
    return "夹取视觉验证目标缺失";
  }
  if (reason == "grab_verify_target_still_visible") {
    return "夹取视觉验证超时后原目标仍可见";
  }
  if (reason == "grab_verify_no_new_frame") {
    return "夹取视觉验证期间没有新视觉帧";
  }
  if (reason == "grab_verify_target_not_stably_lost") {
    return "夹取视觉验证超时且未达到连续消失帧";
  }
  if (reason == "grab_retry_context_missing") {
    return "夹取失败重试上下文缺失";
  }
  if (reason == "grab_retry_center_start_failed") {
    return "夹取失败重试前格中心归位启动失败";
  }
  if (reason == "kfs_visual_align_total_timeout") {
    return "KFS 视觉横移对齐总超时";
  }
  if (reason == "arm_high_raise_failed") {
    return "ARM_HIGH_RAISE 命令或完成反馈失败";
  }
  if (reason == "entry_arm_raise_failed") {
    return "入口 ARM_RAISE 命令或完成反馈失败";
  }
  if (reason == "fake_avoid_arm_raise_failed") {
    return "假 KFS 避障 ARM_RAISE 命令或完成反馈失败";
  }
  if (reason == "fake_avoid_arm_lower_failed") {
    return "假 KFS 避障 ARM_LOWER 命令或完成反馈失败";
  }
  if (reason == "transition_arm_raise_failed") {
    return "格间转换 ARM_RAISE 命令或完成反馈失败";
  }
  if (reason == "transition_arm_lower_failed") {
    return "格间转换 ARM_LOWER 命令或完成反馈失败";
  }
  if (reason == "row4_arm_lower_failed") {
    return "第四行 ARM_LOWER 命令或完成反馈失败";
  }
  if (reason == "second_arm_lower_failed" ||
      reason == "kfs_second_arm_lower_failed") {
    return "KFS 夹取前第二节机械臂下降命令或完成反馈失败";
  }
  if (reason == "front_pushrod_extend_failed") {
    return "FRONT_PUSHROD_EXTEND 命令失败";
  }
  if (reason == "rear_pushrod_retract_failed") {
    return "REAR_PUSHROD_RETRACT 命令失败";
  }
  if (reason == "rear_pushrod_extend_failed") {
    return "REAR_PUSHROD_EXTEND 命令失败";
  }
  if (reason == "front_pushrod_retract_failed") {
    return "FRONT_PUSHROD_RETRACT 命令失败";
  }
  if (reason == "climb_pair_command_failed") {
    return "上阶梯前推杆收回和后推杆伸出并发命令失败";
  }
  if (reason == "descend_pair_command_failed") {
    return "下阶梯后推杆收回和前推杆伸出并发命令失败";
  }
  if (reason == "kfs_odom_motion_not_started") {
    return "KFS odom 闭环运动尚未启动";
  }
  if (startsWith(reason, "move_timeout_")) {
    return "相对移动超时，段=" + suffixAfter(reason, "move_timeout_");
  }
  if (startsWith(reason, "turn_timeout_")) {
    return "转向超时，段=" + suffixAfter(reason, "turn_timeout_");
  }
  if (startsWith(reason, "center_align_timeout_")) {
    return "格中心归位超时，段=" + suffixAfter(reason, "center_align_timeout_");
  }
  if (startsWith(reason, "kfs_odom_motion_timeout_")) {
    return "KFS odom 闭环运动超时，段=" +
           suffixAfter(reason, "kfs_odom_motion_timeout_");
  }
  if (startsWith(reason, "kfs_odom_motion_speed_non_positive_")) {
    return "KFS odom 闭环速度配置非正，段=" +
           suffixAfter(reason, "kfs_odom_motion_speed_non_positive_");
  }
  return reason;
}

std::string translateR2LockRejectReason(const std::string &reason) {
  if (reason == "pickup_limit_reached") {
    return "夹取数量已达上限";
  }
  if (reason == "vision_not_running") {
    return "视觉运行时未运行";
  }
  if (reason == "snapshot_invalid") {
    return "视觉快照无效";
  }
  if (reason == "no_label_match") {
    return "没有匹配R2目标标签的检测框";
  }
  if (reason == "ignored_target") {
    return "目标已在忽略列表中";
  }
  if (reason == "depth_invalid") {
    return "深度采样失败";
  }
  if (reason == "selection_failed") {
    return "目标选择失败";
  }
  return reason.empty() ? "未知原因" : reason;
}

// 视觉配置参数既允许传绝对路径，也允许传 rc26_vision share 目录下的相对路径。
// 这里把“空值”解析为 rc26_vision/config/vision_models.yaml，便于 bringup YAML 只
// 维护模型 ID，而不需要每台机器重复写包安装路径。
std::string resolveVisionConfig(const std::string &configured) {
  namespace fs = std::filesystem;
  if (!configured.empty() && fs::exists(configured)) {
    return fs::path(configured).lexically_normal().string();
  }
  try {
    const fs::path share =
        ament_index_cpp::get_package_share_directory("rc26_vision");
    const fs::path candidate =
        configured.empty() ? (share / "config" / "vision_models.yaml")
                           : (share / configured);
    if (fs::exists(candidate)) {
      return candidate.lexically_normal().string();
    }
  } catch (...) {
  }
  return configured;
}

// ROS2 YAML 参数里的字符串列表常被人工编辑；这里统一裁剪空白并丢弃空项，
// 避免因为 " T_ " 或空字符串把标签匹配变成隐式放行。
std::vector<std::string> sanitized(std::vector<std::string> values) {
  std::vector<std::string> out;
  out.reserve(values.size());
  for (auto &value : values) {
    const auto first = value.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) {
      continue;
    }
    const auto last = value.find_last_not_of(" \t\r\n");
    const auto clean = value.substr(first, last - first + 1);
    if (!clean.empty()) {
      out.push_back(clean);
    }
  }
  return out;
}

// MF 离散格按三列展开：grid1..3 为第一行，grid4..6 为第二行。
// transitionYaw() 会用行列差把离散边转换为底盘目标 yaw。
bool validGrid(int grid_id) { return grid_id >= 1 && grid_id <= 12; }
int gridRow(int grid_id) { return (grid_id - 1) / 3; }
int gridCol(int grid_id) { return (grid_id - 1) % 3; }

double finiteAbsOr(double value, double fallback) {
  return std::isfinite(value) ? std::abs(value) : fallback;
}

double normalizedAngle(double angle_rad) {
  while (angle_rad > kPi) {
    angle_rad -= 2.0 * kPi;
  }
  while (angle_rad < -kPi) {
    angle_rad += 2.0 * kPi;
  }
  return angle_rad;
}

std::string detectionSummary(const rc26_vision::Detection &det) {
  std::ostringstream oss;
  oss << rc26_vision::visualTargetLabel(det) << "/class=" << det.class_id
      << "/score=" << det.score << "/bbox=[" << det.x1 << " " << det.y1
      << " " << det.x2 << " " << det.y2 << "]";
  return oss.str();
}

std::string detectionsSummary(const std::vector<rc26_vision::Detection> &dets,
                              std::size_t limit = 5) {
  if (dets.empty()) {
    return "[]";
  }
  std::ostringstream oss;
  oss << "[";
  const std::size_t count = std::min(limit, dets.size());
  for (std::size_t i = 0; i < count; ++i) {
    if (i > 0) {
      oss << "; ";
    }
    oss << detectionSummary(dets[i]);
  }
  if (dets.size() > count) {
    oss << "; ...共" << dets.size() << "个";
  }
  oss << "]";
  return oss.str();
}

std::string intVectorSummary(const std::vector<int> &values) {
  if (values.empty()) {
    return "[]";
  }
  std::ostringstream oss;
  oss << "[";
  for (std::size_t i = 0; i < values.size(); ++i) {
    if (i > 0) {
      oss << ",";
    }
    oss << values[i];
  }
  oss << "]";
  return oss.str();
}

// pickup_source_ 记录“第一次在入口侧夹到 KFS 时来自哪条阶梯”，后续假 KFS
// 避障会用它决定从 1 号侧还是 3 号侧绕行。
const char *sourceName(MfPreselectionPickupSource source) {
  switch (source) {
  case MfPreselectionPickupSource::Stair1:
    return "1号入口";
  case MfPreselectionPickupSource::Stair2:
    return "2号入口";
  case MfPreselectionPickupSource::Stair3:
    return "3号入口";
  case MfPreselectionPickupSource::None:
  default:
    return "无";
  }
}

} // namespace

bool MfPreselectionLogicResult::labelMatches(
    const std::string &label, const std::vector<std::string> &exact_labels,
    const std::vector<std::string> &prefixes) {
  // 标签匹配采用“精确标签优先 + 前缀兜底”的双配置。这样既能兼容
  // R_R1/B_R1 这类固定标签，也能支持 T_*、F_* 这类按模型扩展的系列标签。
  for (const auto &exact : exact_labels) {
    if (!exact.empty() && label == exact) {
      return true;
    }
  }
  for (const auto &prefix : prefixes) {
    if (!prefix.empty() && label.rfind(prefix, 0) == 0) {
      return true;
    }
  }
  return false;
}

bool MfPreselectionLogicResult::r1KfsScoreAccepted(
    const std::string &label, double score, double min_score) {
  if (label != "R1_KFS") {
    return true;
  }
  const double normalized_min_score =
      std::clamp(std::isfinite(min_score) ? min_score : 0.50, 0.0, 1.0);
  return std::isfinite(score) && score >= normalized_min_score;
}

bool MfPreselectionLogicResult::canPickup(int pickup_count,
                                          int max_pickup_count) {
  // max_pickup_count 允许配置为 0，用于现场只跑避障/路线、不触发夹取的测试。
  return pickup_count < std::max(0, max_pickup_count);
}

MfPreselectionLogicResult::GrabRetryAction
MfPreselectionLogicResult::grabRetryAction(
    bool target_still_visible, MfPreselectionPickupSource source,
    bool entry_high_protocol, bool path_blocking) {
  if (!target_still_visible) {
    return GrabRetryAction::None;
  }
  if (entry_high_protocol || source != MfPreselectionPickupSource::None) {
    return GrabRetryAction::EntryBackoff;
  }
  return path_blocking ? GrabRetryAction::GridCenterRetry
                       : GrabRetryAction::None;
}

bool MfPreselectionLogicResult::mandatoryEntryStair2Retry(
    MfPreselectionPickupSource source, bool entry_high_protocol) {
  return source == MfPreselectionPickupSource::Stair2 && !entry_high_protocol;
}

bool MfPreselectionLogicResult::entryInterruptOffsetAcceptable(
    int offset_px, const MfPreselectionParams &params) {
  return std::abs(offset_px) <= std::max(0, params.entry_interrupt_max_offset_px);
}

bool MfPreselectionLogicResult::entryInterruptOffsetAcceptable(
    int offset_px, double lateral_speed_mps, double depth_m,
    const MfPreselectionParams &params) {
  return std::abs(offset_px) <=
         entryInterruptEffectiveOffsetLimitPx(lateral_speed_mps, depth_m,
                                              params);
}

double MfPreselectionLogicResult::mcuSineStopTime(double speed_mps,
                                                  double acc_mps2) {
  if (!std::isfinite(speed_mps) || !std::isfinite(acc_mps2) ||
      acc_mps2 <= 0.0) {
    return 0.0;
  }
  return kPi * std::abs(speed_mps) / (2.0 * acc_mps2);
}

double MfPreselectionLogicResult::mcuSineStopDistance(double speed_mps,
                                                      double acc_mps2) {
  if (!std::isfinite(speed_mps) || !std::isfinite(acc_mps2) ||
      acc_mps2 <= 0.0) {
    return 0.0;
  }
  const double speed = std::abs(speed_mps);
  return kPi * speed * speed / (4.0 * acc_mps2);
}

double MfPreselectionLogicResult::entryReturnToCenterDistanceCompensation(
    double lateral_speed_mps, const MfPreselectionParams &params) {
  const double speed =
      std::isfinite(lateral_speed_mps) ? std::abs(lateral_speed_mps) : 0.0;
  if (speed <= 0.0) {
    return 0.0;
  }
  const double stop_distance =
      mcuSineStopDistance(speed, params.entry_mcu_vy_acc_mps2);
  const double latency =
      std::max(0.0, std::isfinite(params.entry_interrupt_latency_s)
                        ? params.entry_interrupt_latency_s
                        : 0.0);
  return stop_distance + speed * latency;
}

double MfPreselectionLogicResult::entryReturnToCenterCompensatedDistance(
    double raw_distance_m, double lateral_speed_mps,
    const MfPreselectionParams &params, double &compensation_m) {
  const double raw =
      std::isfinite(raw_distance_m) ? std::max(0.0, raw_distance_m) : 0.0;
  compensation_m =
      entryReturnToCenterDistanceCompensation(lateral_speed_mps, params);
  return std::max(0.0, raw - compensation_m);
}

int MfPreselectionLogicResult::entryInterruptDynamicExtraPx(
    double lateral_speed_mps, double depth_m,
    const MfPreselectionParams &params) {
  if (!params.entry_interrupt_dynamic_comp_enable ||
      !std::isfinite(depth_m) || depth_m <= 0.0 ||
      !std::isfinite(params.entry_interrupt_fx_px) ||
      params.entry_interrupt_fx_px <= 0.0) {
    return 0;
  }

  const double speed = std::isfinite(lateral_speed_mps)
                           ? std::abs(lateral_speed_mps)
                           : 0.0;
  const double stop_distance =
      mcuSineStopDistance(speed, params.entry_mcu_vy_acc_mps2);
  const double latency =
      std::max(0.0, std::isfinite(params.entry_interrupt_latency_s)
                        ? params.entry_interrupt_latency_s
                        : 0.0);
  const double compensated_m = stop_distance + speed * latency;
  if (compensated_m <= 0.0) {
    return 0;
  }

  const int raw_extra_px = static_cast<int>(
      std::lround(compensated_m * params.entry_interrupt_fx_px / depth_m));
  const int min_extra = std::max(0, params.entry_interrupt_extra_px_min);
  const int max_extra = std::max(min_extra, params.entry_interrupt_extra_px_max);
  return std::clamp(raw_extra_px, min_extra, max_extra);
}

int MfPreselectionLogicResult::entryInterruptEffectiveOffsetLimitPx(
    double lateral_speed_mps, double depth_m,
    const MfPreselectionParams &params) {
  return std::max(0, params.entry_interrupt_max_offset_px) +
         entryInterruptDynamicExtraPx(lateral_speed_mps, depth_m, params);
}

double MfPreselectionLogicResult::entryMcuStopSettleDuration(
    double lateral_speed_mps, const MfPreselectionParams &params) {
  if (!params.entry_mcu_stop_settle_enable) {
    return 0.0;
  }
  const double stop_time =
      mcuSineStopTime(lateral_speed_mps, params.entry_mcu_vy_acc_mps2);
  const double margin =
      std::max(0.0, std::isfinite(params.entry_mcu_stop_margin_s)
                        ? params.entry_mcu_stop_margin_s
                        : 0.0);
  const double max_wait =
      std::max(0.0, std::isfinite(params.entry_mcu_stop_max_wait_s)
                        ? params.entry_mcu_stop_max_wait_s
                        : 0.0);
  const double wait = stop_time + margin;
  if (wait <= 0.0) {
    return 0.0;
  }
  return max_wait > 0.0 ? std::min(wait, max_wait) : wait;
}

bool MfPreselectionLogicResult::kfsAlignTimeoutPickupAllowed(
    int offset_px, bool has_depth, const MfPreselectionParams &params) {
  return has_depth && std::abs(offset_px) <=
                          std::max(0, params.kfs_align_timeout_pickup_tolerance_px);
}

MfPreselectionLogicResult::DepthRoiDiagnostic
MfPreselectionLogicResult::depthRoiDiagnostic(
    const cv::Mat &depth, int cx, int cy,
    const rc26_vision::DepthRoiSamplerConfig &config) {
  DepthRoiDiagnostic diagnostic;
  diagnostic.cx = cx;
  diagnostic.cy = cy;
  diagnostic.roi_size = config.roi_size;
  diagnostic.min_valid_count = config.min_valid_count;
  diagnostic.depth_rows = depth.rows;
  diagnostic.depth_cols = depth.cols;
  diagnostic.depth_type = depth.type();
  diagnostic.depth_type_name = matTypeName(depth);
  diagnostic.min_depth_m = config.min_depth_m;
  diagnostic.max_depth_m = config.max_depth_m;

  if (depth.empty() || depth.rows <= 0 || depth.cols <= 0) {
    diagnostic.depth_empty = true;
    diagnostic.primary_failure = "深度图为空";
    return diagnostic;
  }
  if (depth.channels() != 1 || config.roi_size <= 0 ||
      config.min_valid_count <= 0 ||
      (depth.type() != CV_16UC1 && depth.type() != CV_32FC1)) {
    diagnostic.unsupported_type = true;
    diagnostic.primary_failure = "深度类型不支持";
    return diagnostic;
  }

  const int half = config.roi_size / 2;
  const int clamped_cx = std::clamp(cx, 0, depth.cols - 1);
  const int clamped_cy = std::clamp(cy, 0, depth.rows - 1);
  diagnostic.cx = clamped_cx;
  diagnostic.cy = clamped_cy;

  const int x0 = std::max(0, clamped_cx - half);
  const int x1 = std::min(depth.cols - 1, clamped_cx + half);
  const int y0 = std::max(0, clamped_cy - half);
  const int y1 = std::min(depth.rows - 1, clamped_cy + half);
  if (x0 > x1 || y0 > y1) {
    diagnostic.primary_failure = "ROI无有效原始深度";
    return diagnostic;
  }

  std::vector<double> raw_samples;
  std::vector<double> window_samples;
  raw_samples.reserve(static_cast<std::size_t>((x1 - x0 + 1) * (y1 - y0 + 1)));
  window_samples.reserve(raw_samples.capacity());

  for (int y = y0; y <= y1; ++y) {
    for (int x = x0; x <= x1; ++x) {
      ++diagnostic.total_pixels;
      double z_m = 0.0;
      if (depth.type() == CV_16UC1) {
        const uint16_t z_mm = depth.at<uint16_t>(y, x);
        if (z_mm == 0U) {
          ++diagnostic.zero_depth_count;
          continue;
        }
        z_m = static_cast<double>(z_mm) * 1e-3;
      } else {
        const float z = depth.at<float>(y, x);
        if (!std::isfinite(z) || z <= 0.0F) {
          ++diagnostic.non_finite_count;
          continue;
        }
        z_m = static_cast<double>(z);
      }

      raw_samples.push_back(z_m);
      if (z_m < config.min_depth_m) {
        ++diagnostic.below_min_count;
        continue;
      }
      if (z_m > config.max_depth_m) {
        ++diagnostic.above_max_count;
        continue;
      }
      window_samples.push_back(z_m);
    }
  }

  diagnostic.raw_valid_count = static_cast<int>(raw_samples.size());
  if (!raw_samples.empty()) {
    diagnostic.raw_min_m =
        *std::min_element(raw_samples.begin(), raw_samples.end());
    diagnostic.raw_max_m =
        *std::max_element(raw_samples.begin(), raw_samples.end());
    auto mid = raw_samples.begin() +
               static_cast<std::ptrdiff_t>(raw_samples.size() / 2);
    std::nth_element(raw_samples.begin(), mid, raw_samples.end());
    diagnostic.raw_median_m = *mid;
  }

  diagnostic.window_valid_count = static_cast<int>(window_samples.size());
  if (diagnostic.window_valid_count >= config.min_valid_count) {
    auto mid = window_samples.begin() +
               static_cast<std::ptrdiff_t>(window_samples.size() / 2);
    std::nth_element(window_samples.begin(), mid, window_samples.end());
    diagnostic.sampled = true;
    diagnostic.sampled_depth_m = *mid;
    diagnostic.primary_failure.clear();
    return diagnostic;
  }

  if (diagnostic.raw_valid_count <= 0) {
    diagnostic.primary_failure = "ROI无有效原始深度";
  } else if (diagnostic.below_min_count > 0 &&
             diagnostic.below_min_count >= diagnostic.above_max_count &&
             diagnostic.window_valid_count == 0) {
    diagnostic.primary_failure = "有效深度低于窗口";
  } else if (diagnostic.above_max_count > 0 &&
             diagnostic.above_max_count >= diagnostic.below_min_count &&
             diagnostic.window_valid_count == 0) {
    diagnostic.primary_failure = "有效深度高于窗口";
  } else {
    diagnostic.primary_failure = "窗口内有效点不足";
  }
  return diagnostic;
}

std::string MfPreselectionLogicResult::depthRoiDiagnosticDetail(
    const DepthRoiDiagnostic &diagnostic) {
  std::ostringstream detail;
  detail << "深度失败主因="
         << (diagnostic.primary_failure.empty() ? "无"
                                                : diagnostic.primary_failure)
         << " 深度采样点=(" << diagnostic.cx << "," << diagnostic.cy << ")"
         << " ROI=" << diagnostic.roi_size
         << " 最小有效点=" << diagnostic.min_valid_count
         << " 深度类型=" << diagnostic.depth_type_name
         << " 深度尺寸=" << diagnostic.depth_cols << "x"
         << diagnostic.depth_rows
         << " 深度窗口=[" << diagnostic.min_depth_m << ","
         << diagnostic.max_depth_m << "]"
         << " ROI总点=" << diagnostic.total_pixels
         << " 零深度点=" << diagnostic.zero_depth_count
         << " 非有限或非正点=" << diagnostic.non_finite_count
         << " 低于窗口点=" << diagnostic.below_min_count
         << " 高于窗口点=" << diagnostic.above_max_count
         << " 窗口内有效点=" << diagnostic.window_valid_count
         << " 原始有效点=" << diagnostic.raw_valid_count;
  if (diagnostic.raw_valid_count > 0) {
    detail << " raw_min=" << diagnostic.raw_min_m
           << " raw_median=" << diagnostic.raw_median_m
           << " raw_max=" << diagnostic.raw_max_m;
  }
  if (diagnostic.sampled) {
    detail << " sampled_depth=" << diagnostic.sampled_depth_m;
  }
  return detail.str();
}

const char *MfPreselectionLogicResult::kfsDepthSourceText(
    KfsDepthSource source) {
  switch (source) {
  case KfsDepthSource::CenterRoi:
    return "中心ROI";
  case KfsDepthSource::BboxMultiRoi:
    return "bbox多点ROI";
  case KfsDepthSource::MonocularBbox:
    return "尺寸估距";
  case KfsDepthSource::None:
  default:
    return "无";
  }
}

bool MfPreselectionLogicResult::kfsDepthSourceIsReal(KfsDepthSource source) {
  return source == KfsDepthSource::CenterRoi ||
         source == KfsDepthSource::BboxMultiRoi;
}

MfPreselectionLogicResult::KfsBboxDepthSample
MfPreselectionLogicResult::sampleKfsDepthFromBbox(
    const cv::Mat &depth, double x1, double y1, double x2, double y2,
    const rc26_vision::DepthRoiSamplerConfig &config,
    const std::vector<double> &sample_ratios, int min_success_count) {
  KfsBboxDepthSample result;
  const double left = std::min(x1, x2);
  const double right = std::max(x1, x2);
  const double top = std::min(y1, y2);
  const double bottom = std::max(y1, y2);
  std::vector<double> ratios;
  ratios.reserve(sample_ratios.size());
  for (const double ratio : sample_ratios) {
    if (std::isfinite(ratio)) {
      ratios.push_back(std::clamp(ratio, 0.0, 1.0));
    }
  }
  if (ratios.empty()) {
    ratios = {0.25, 0.50, 0.75};
  }
  std::sort(ratios.begin(), ratios.end());
  ratios.erase(std::unique(ratios.begin(), ratios.end()), ratios.end());
  const int required_success_count =
      std::clamp(min_success_count, 1,
                 static_cast<int>(ratios.size() * ratios.size()));
  std::vector<double> samples;
  std::optional<DepthRoiDiagnostic> representative;

  for (const double yr : ratios) {
    for (const double xr : ratios) {
      const int cx = static_cast<int>(std::lround(left + (right - left) * xr));
      const int cy = static_cast<int>(std::lround(top + (bottom - top) * yr));
      ++result.sample_point_count;
      const auto sampled =
          rc26_vision::sampleMedianDepth(depth, cx, cy, config);
      const auto diagnostic = depthRoiDiagnostic(depth, cx, cy, config);
      if (sampled.has_value()) {
        ++result.success_count;
        samples.push_back(*sampled);
      } else {
        const bool replace =
            !representative.has_value() ||
            diagnostic.window_valid_count >
                representative->window_valid_count ||
            (diagnostic.window_valid_count ==
                 representative->window_valid_count &&
             diagnostic.raw_valid_count > representative->raw_valid_count);
        if (replace) {
          representative = diagnostic;
        }
      }
    }
  }

  if (static_cast<int>(samples.size()) >= required_success_count) {
    auto mid = samples.begin() +
               static_cast<std::ptrdiff_t>(samples.size() / 2);
    std::nth_element(samples.begin(), mid, samples.end());
    result.has_depth = true;
    result.depth_m = *mid;
    result.source = result.success_count == 1 && samples.size() == 1
                        ? KfsDepthSource::CenterRoi
                        : KfsDepthSource::BboxMultiRoi;
    if (result.success_count == 1) {
      const int center_cx =
          static_cast<int>(std::lround((left + right) * 0.5));
      const int center_cy =
          static_cast<int>(std::lround((top + bottom) * 0.5));
      const auto center_sample =
          rc26_vision::sampleMedianDepth(depth, center_cx, center_cy, config);
      result.source = center_sample.has_value()
                          ? KfsDepthSource::CenterRoi
                          : KfsDepthSource::BboxMultiRoi;
    }
  } else if (representative.has_value()) {
    result.representative_failure = *representative;
  } else {
    result.representative_failure =
        depthRoiDiagnostic(depth, static_cast<int>(std::lround((left + right) * 0.5)),
                           static_cast<int>(std::lround((top + bottom) * 0.5)),
                           config);
  }

  std::ostringstream detail;
  detail << "bbox采样点数=" << result.sample_point_count
         << " bbox采样成功数=" << result.success_count
         << " bbox最少成功点=" << required_success_count
         << " bbox采样比例数=" << ratios.size();
  if (result.has_depth) {
    detail << " bbox采样深度=" << result.depth_m
           << " depth_source=" << kfsDepthSourceText(result.source);
  } else if (!samples.empty()) {
    detail << " 深度失败主因=窗口内有效点不足"
           << " 成功点未达阈值 sampled_success=" << samples.size()
           << " required_success=" << required_success_count;
  } else {
    detail << " "
           << depthRoiDiagnosticDetail(result.representative_failure);
  }
  result.detail = detail.str();
  return result;
}

MfPreselectionLogicResult::KfsMonocularDepthEstimate
MfPreselectionLogicResult::estimateKfsMonocularDepth(
    double bbox_width_px, double bbox_height_px, double locked_depth_m,
    const MfPreselectionParams &params, double min_depth_m,
    double max_depth_m) {
  KfsMonocularDepthEstimate estimate;
  estimate.bbox_width_px = bbox_width_px;
  estimate.bbox_height_px = bbox_height_px;

  const auto reject = [&estimate](const std::string &reason) {
    estimate.reject_reason = reason;
    std::ostringstream detail;
    detail << "尺寸估距可用=否 尺寸估距拒绝原因=" << reason
           << " bbox_w=" << estimate.bbox_width_px
           << " bbox_h=" << estimate.bbox_height_px
           << " z_width=" << estimate.z_width_m
           << " z_height=" << estimate.z_height_m
           << " chosen_z=" << estimate.depth_m
           << " locked_delta=" << estimate.locked_delta_m;
    estimate.detail = detail.str();
  };

  if (!params.kfs_mono_distance_fallback_enable) {
    reject("尺寸估距未启用");
    return estimate;
  }
  if (!std::isfinite(locked_depth_m) || locked_depth_m <= 0.0) {
    reject("无真实锁定深度");
    return estimate;
  }
  const double min_bbox_px =
      static_cast<double>(std::max(1, params.kfs_mono_min_bbox_px));
  if (!std::isfinite(bbox_width_px) || !std::isfinite(bbox_height_px) ||
      bbox_width_px < min_bbox_px || bbox_height_px < min_bbox_px) {
    reject("bbox尺寸过小");
    return estimate;
  }
  if (!std::isfinite(params.kfs_mono_target_width_m) ||
      !std::isfinite(params.kfs_mono_target_height_m) ||
      !std::isfinite(params.kfs_mono_fx_px) ||
      !std::isfinite(params.kfs_mono_fy_px) ||
      params.kfs_mono_target_width_m <= 0.0 ||
      params.kfs_mono_target_height_m <= 0.0 ||
      params.kfs_mono_fx_px <= 0.0 || params.kfs_mono_fy_px <= 0.0) {
    reject("尺寸或内参非法");
    return estimate;
  }

  estimate.z_width_m =
      params.kfs_mono_fx_px * params.kfs_mono_target_width_m / bbox_width_px;
  estimate.z_height_m =
      params.kfs_mono_fy_px * params.kfs_mono_target_height_m / bbox_height_px;
  if (!std::isfinite(estimate.z_width_m) ||
      !std::isfinite(estimate.z_height_m) || estimate.z_width_m <= 0.0 ||
      estimate.z_height_m <= 0.0) {
    reject("尺寸估距结果非法");
    return estimate;
  }

  estimate.depth_m = std::min(estimate.z_width_m, estimate.z_height_m);
  if (estimate.depth_m < min_depth_m || estimate.depth_m > max_depth_m) {
    reject("尺寸估距超出深度窗口");
    return estimate;
  }

  estimate.locked_delta_m = std::abs(estimate.depth_m - locked_depth_m);
  const double max_delta =
      std::max(0.0, params.kfs_mono_max_delta_from_locked_m);
  if (estimate.locked_delta_m > max_delta) {
    reject("尺寸估距偏离真实锁定深度过大");
    return estimate;
  }

  estimate.usable = true;
  std::ostringstream detail;
  detail << "尺寸估距可用=是"
         << " bbox_w=" << estimate.bbox_width_px
         << " bbox_h=" << estimate.bbox_height_px
         << " fx=" << params.kfs_mono_fx_px
         << " fy=" << params.kfs_mono_fy_px
         << " target_w=" << params.kfs_mono_target_width_m
         << " target_h=" << params.kfs_mono_target_height_m
         << " z_width=" << estimate.z_width_m
         << " z_height=" << estimate.z_height_m
         << " chosen_z=" << estimate.depth_m
         << " locked_delta=" << estimate.locked_delta_m;
  estimate.detail = detail.str();
  return estimate;
}

rc26_vision::TipAlignmentConfig MfPreselectionLogicResult::kfsAlignmentConfig(
    const MfPreselectionParams &params, double target_yaw_rad) {
  rc26_vision::TipAlignmentConfig config;
  config.target_lock_enable = true;
  config.target_lock_max_jump_px = params.kfs_align_max_jump_px;
  config.lost_stop_frames = params.kfs_lost_stop_frames;
  config.tolerance_px = params.kfs_align_tolerance_px;
  config.target_line_offset_px = params.kfs_align_target_line_offset_px;
  config.kp = params.kfs_align_kp;
  config.min_speed_mps = params.kfs_align_min_speed_mps;
  config.max_speed_mps = params.kfs_align_max_speed_mps;
  config.invert_direction = params.kfs_invert_lateral_direction;
  config.heading_hold_enable = true;
  config.target_yaw_rad = target_yaw_rad;
  config.heading_kp = params.heading_kp;
  config.heading_max_speed_radps = params.heading_max_speed_radps;
  config.heading_tolerance_rad =
      std::abs(params.kfs_odom_yaw_tolerance_deg) * kDeg2Rad;
  config.heading_gate_rad =
      std::max(config.heading_tolerance_rad,
               std::abs(params.kfs_align_heading_gate_deg) * kDeg2Rad);
  return config;
}

double MfPreselectionLogicResult::kfsOpenLoopDistance(
    double locked_depth_m, double grab_distance_m) {
  return std::max(0.0, locked_depth_m - grab_distance_m);
}

double MfPreselectionLogicResult::kfsOpenLoopDuration(double distance_m,
                                                      double speed_mps) {
  if (distance_m <= 0.0) {
    return 0.0;
  }
  if (speed_mps <= 0.0) {
    return std::numeric_limits<double>::infinity();
  }
  return distance_m / speed_mps;
}

double MfPreselectionLogicResult::kfsApproachOdomDistance(
    double locked_depth_m, const MfPreselectionParams &params) {
  const double sign = params.kfs_approach_x_sign < 0 ? -1.0 : 1.0;
  return sign *
         kfsOpenLoopDistance(locked_depth_m, params.kfs_grab_distance_m);
}

void MfPreselectionLogicResult::normalizeKfsOdomParams(
    MfPreselectionParams &params) {
  params.kfs_align_tolerance_px =
      std::max(0, params.kfs_align_tolerance_px);
  params.kfs_align_target_line_offset_px =
      std::clamp(params.kfs_align_target_line_offset_px, -10000, 10000);
  params.kfs_odom_xy_kp =
      std::isfinite(params.kfs_odom_xy_kp) && params.kfs_odom_xy_kp > 0.0
          ? params.kfs_odom_xy_kp
          : kDefaultKfsOdomXyKp;
  params.kfs_approach_odom_tolerance_m =
      std::isfinite(params.kfs_approach_odom_tolerance_m) &&
              params.kfs_approach_odom_tolerance_m > 0.0
          ? std::abs(params.kfs_approach_odom_tolerance_m)
          : kDefaultKfsApproachOdomToleranceM;
  params.kfs_odom_yaw_tolerance_deg =
      std::isfinite(params.kfs_odom_yaw_tolerance_deg) &&
              params.kfs_odom_yaw_tolerance_deg >= 0.0
          ? params.kfs_odom_yaw_tolerance_deg
          : kDefaultKfsOdomYawToleranceDeg;
  params.kfs_odom_stable_ticks =
      std::max(1, params.kfs_odom_stable_ticks);
  params.kfs_align_max_jump_px =
      std::max(0, params.kfs_align_max_jump_px);
  if (!std::isfinite(params.kfs_approach_min_speed_mps) ||
      params.kfs_approach_min_speed_mps < 0.0) {
    params.kfs_approach_min_speed_mps = kDefaultKfsApproachMinSpeedMps;
  }
  params.kfs_approach_min_speed_mps =
      std::min(std::abs(params.kfs_approach_min_speed_mps),
               std::abs(params.kfs_approach_speed_mps));
}

double MfPreselectionLogicResult::fakeAvoidanceYaw(
    MfPreselectionPickupSource source, const MfPreselectionParams &params) {
  // 假 KFS 避障方向取决于入口夹取来源：从 3 号侧拿到目标时向 3 号侧绕，
  // 其它情况默认走 1 号侧方向。None 也归到 1 号侧作为保守兜底。
  if (source == MfPreselectionPickupSource::Stair3) {
    return params.stair3_direction_yaw_rad;
  }
  return params.stair1_direction_yaw_rad;
}

uint8_t MfPreselectionLogicResult::grabCommandForHighSide(
    bool high_side, const MfPreselectionParams &params) {
  // high_side 表示当前 KFS 位于本次台阶动作的高侧：上台阶或高抬升入口用
  // GRAB_KFS_UP，下台阶观察到的目标用 GRAB_KFS_DOWN。
  const int value =
      high_side ? params.grab_kfs_up_command_id : params.grab_kfs_down_command_id;
  return static_cast<uint8_t>(std::clamp(value, 0, 255));
}

uint8_t MfPreselectionLogicResult::grabCommandForPickup(
    bool high_side, MfPreselectionPickupSource source,
    bool entry_high_protocol, const MfPreselectionParams &params) {
  (void)source;
  if (high_side && entry_high_protocol) {
    return static_cast<uint8_t>(
        std::clamp(params.entry_grab_kfs_up_command_id, 0, 255));
  }
  return grabCommandForHighSide(high_side, params);
}

int MfPreselectionLogicResult::grabDoneFeedbackForPickup(
    bool high_side, MfPreselectionPickupSource source,
    bool entry_high_protocol, const MfPreselectionParams &params) {
  (void)source;
  if (high_side && entry_high_protocol) {
    return static_cast<uint8_t>(
        std::clamp(params.entry_grab_kfs_up_done_feedback_id, 0, 255));
  }
  return -1;
}

bool MfPreselectionLogicResult::postGrabCenterAlignRequired(
    MfPreselectionPickupSource source, bool entry_high_protocol) {
  return source == MfPreselectionPickupSource::None && !entry_high_protocol;
}

double MfPreselectionLogicResult::bboxIou(
    const MfPreselectionTargetSnapshot &a,
    const MfPreselectionTargetSnapshot &b) {
  return rc26_vision::bboxIou(a, b);
}

bool MfPreselectionLogicResult::isSameVisualTarget(
    const MfPreselectionTargetSnapshot &reference,
    const MfPreselectionTargetSnapshot &candidate, double iou_threshold) {
  return rc26_vision::isSameVisualTarget(reference, candidate, iou_threshold);
}

bool MfPreselectionLogicResult::isIgnoredTarget(
    const MfPreselectionTargetSnapshot &candidate,
    const std::vector<MfPreselectionTargetSnapshot> &ignored_targets,
    double iou_threshold) {
  return rc26_vision::isIgnoredVisualTarget(candidate, ignored_targets,
                                           iou_threshold);
}

std::optional<int> MfPreselectionLogicResult::fakeAvoidanceTargetGrid(
    int current_grid, MfPreselectionPickupSource source, int mirror_sign) {
  if (!validGrid(current_grid)) {
    return std::nullopt;
  }
  const int side_sign = normalizedMirrorSign(mirror_sign);
  const int col = gridCol(current_grid);
  int target_col = col - side_sign;
  if (source == MfPreselectionPickupSource::Stair3) {
    target_col = col + side_sign;
  }
  if (target_col < 0 || target_col > 2) {
    return std::nullopt;
  }
  return gridRow(current_grid) * 3 + target_col + 1;
}

std::optional<int>
MfPreselectionLogicResult::fakeAvoidanceForwardTargetGrid(int current_grid) {
  if (!validGrid(current_grid)) {
    return std::nullopt;
  }
  const int col = gridCol(current_grid);
  if (col != 0 && col != 2) {
    return std::nullopt;
  }
  const int next_grid = current_grid + 3;
  if (!validGrid(next_grid)) {
    return std::nullopt;
  }
  return next_grid;
}

MfPreselectionPickupSource
MfPreselectionLogicResult::entryPickupSourceForLateralOffset(
    double lateral_offset_m, double tolerance_m, int mirror_sign) {
  const double tolerance = std::max(0.0, std::abs(tolerance_m));
  if (!std::isfinite(lateral_offset_m) ||
      std::abs(lateral_offset_m) <= tolerance) {
    return MfPreselectionPickupSource::Stair2;
  }
  const double mirrored_offset =
      lateral_offset_m * static_cast<double>(normalizedMirrorSign(mirror_sign));
  return mirrored_offset > 0.0 ? MfPreselectionPickupSource::Stair1
                               : MfPreselectionPickupSource::Stair3;
}

bool MfPreselectionLogicResult::entryReturnToCenterCommand(
    double lateral_offset_m, double tolerance_m, double speed_mps, double &vy,
    double &distance_m) {
  if (!std::isfinite(lateral_offset_m)) {
    return false;
  }
  const double tolerance = std::max(0.0, std::abs(tolerance_m));
  const double abs_offset = std::abs(lateral_offset_m);
  if (abs_offset <= tolerance) {
    vy = 0.0;
    distance_m = 0.0;
    return true;
  }
  const double speed = std::max(kMinSpeed, finiteAbsOr(speed_mps, kMinSpeed));
  vy = lateral_offset_m > 0.0 ? -speed : speed;
  distance_m = abs_offset;
  return true;
}

bool MfPreselectionLogicResult::finalExitCenterTarget(
    double current_center_x, double current_center_y,
    double exit_heading_yaw_rad, double offset_m, double &target_x,
    double &target_y) {
  if (!std::isfinite(current_center_x) || !std::isfinite(current_center_y) ||
      !std::isfinite(exit_heading_yaw_rad)) {
    return false;
  }
  const double offset = std::max(0.0, finiteAbsOr(offset_m, 1.2));
  const double exit_yaw = normalizedAngle(exit_heading_yaw_rad);
  target_x = current_center_x + offset * std::cos(exit_yaw);
  target_y = current_center_y + offset * std::sin(exit_yaw);
  return std::isfinite(target_x) && std::isfinite(target_y);
}

MfPreselectionFlowAction::MfPreselectionFlowAction(
    const std::string &name, const BT::NodeConfig &config)
    : BT::StatefulActionNode(name, config) {}

MfPreselectionFlowAction::~MfPreselectionFlowAction() { releaseRuntime(); }

BT::PortsList MfPreselectionFlowAction::providedPorts() { return {}; }

BT::NodeStatus MfPreselectionFlowAction::onStart() {
  // onStart 只做一次性运行资源装配和流程复位。所有耗时动作都在 onRunning()
  // 通过内部状态机分 tick 推进，保持 BehaviorTree.CPP 的执行线程可取消。
  if (!setupRuntime()) {
    return BT::NodeStatus::FAILURE;
  }
  if (!setupVision()) {
    return fail("vision_start_failed");
  }

  pickup_count_ = 0;
  entry_pickup_done_ = false;
  direct_exit_mode_ = false;
  fake_avoid_forward_mode_ = false;
  arm_high_raised_ = false;
  arm_high_side_ = false;
  pickup_source_ = MfPreselectionPickupSource::None;
  current_grid_ = 2;
  row4_fake_detected_ = false;
  direct_exit_move_active_ = false;
  clearPathR1Wait();
  clearGrabRetryContext();
  pending_grab_commit_ = false;
  pending_grab_source_ = MfPreselectionPickupSource::None;
  pending_grab_target_.reset();
  pending_grab_entry_high_protocol_ = false;
  ignored_r2_targets_.clear();
  grab_success_direct_exit_ = false;
  post_grab_center_next_phase_ = Phase::Done;
  entry_lateral_reference_captured_ = false;
  entry_lateral_reference_x_ = 0.0;
  entry_lateral_reference_y_ = 0.0;
  entry_lateral_reference_yaw_ = entry_heading_yaw_;
  entry_move_interrupted_active_ = false;
  interrupted_entry_move_next_phase_ = Phase::Done;
  interrupted_entry_move_label_.clear();
  interrupted_entry_move_target_offset_m_ = 0.0;
  entry_move_last_interrupt_sequence_ = 0;
  clearKfsVisualPickup();
  grab_verify_lost_count_ = 0;
  grab_verify_last_sequence_ = 0;
  grab_verify_seen_new_frame_ = false;
  grab_verify_visible_logged_ = false;
  grab_verify_last_logged_lost_count_ = 0;
  entry_heading_yaw_ = params_.exit_yaw_rad;
  turn_target_yaw_ = entry_heading_yaw_;
  // 预选赛 XML 可以在本节点前放可选 OdomDriveX/OdomDriveY 分段入口。
  // 进入本节点时按“已在 2 号入口预备姿态”处理，先看正前方是否已经有 R2 可夹取 KFS。
  writeBlackboardState("entry_detect_stair2");
  RCLCPP_INFO(node_->get_logger(),
              "梅林预选赛流程启动：当前位置按 grid2 / 2号入口处理，入口导航已由行为树前置控制，入口heading=%.3frad，最大夹取数=%d",
              entry_heading_yaw_, params_.max_pickup_count);
  beginDetection(DetectMode::Entry2, params_.entry_detect_timeout_s);
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus MfPreselectionFlowAction::onRunning() {
  switch (phase_) {
  // 所有视觉检测阶段共用 tickDetection()：它按 detect_mode_ 区分入口探测、
  // 行前方检测、周身扫描、第四行假 KFS 检测和台阶切换前观察。
  case Phase::EntryDetectStair2:
  case Phase::EntryDetectStair1:
  case Phase::EntryDetectStair3:
  case Phase::RowFrontDetect:
  case Phase::RowScanDetectLeft:
  case Phase::RowScanDetectBack:
  case Phase::Row4DetectFake:
  case Phase::TransitionObserve:
    return tickDetection();

  case Phase::EntryHighRaise:
    // 2 号入口未看到 R2 KFS 后，先把机械臂抬到更高保持态，再横移探测
    // 1/3 号阶梯；如果后续入口侧发现目标，可以直接以高侧姿态夹取。
    beginMechanismCommand(clampByte(params_.arm_high_raise_command_id),
                          "ARM_HIGH_RAISE",
                          clampByte(params_.arm_high_raise_done_feedback_id),
                          Phase::EntryMoveLeft, "arm_high_raise_failed");
    arm_high_raised_ = true;
    arm_high_side_ = true;
    return BT::NodeStatus::RUNNING;

  case Phase::EntryMoveLeft:
    // 从 2 号入口横移到 1 号入口探测。横移距离和速度只来自参数，
    // 左右方向由 team 镜像符号派生，不在这里假设场地绝对坐标。
    beginMoveRelative(0.0, std::abs(params_.lateral_probe_speed_mps) *
                               static_cast<double>(params_.field_mirror_sign),
                      params_.entry_probe_left_distance_m,
                      Phase::EntryDetectStair1, "entry_probe_left");
    return BT::NodeStatus::RUNNING;

  case Phase::EntryReturnFromStair1:
  {
    // 1 号入口夹取后或探测完成后回到中间入口，为上中间列首个台阶做准备。
    const double vy = -std::abs(params_.lateral_probe_speed_mps) *
                      static_cast<double>(params_.field_mirror_sign);
    const double raw_distance_m = params_.entry_probe_return_distance_m;
    double compensation_m = 0.0;
    const double distance_m =
        MfPreselectionLogicResult::entryReturnToCenterCompensatedDistance(
            raw_distance_m, vy, params_, compensation_m);
    if (distance_m <= params_.move_tolerance_m) {
      RCLCPP_INFO(
          node_->get_logger(),
          "梅林预选赛入口1号回2号入口距离经MCU减速+延迟补偿后已无需横移：vy=%.3f raw_distance=%.3fm compensation=%.3fm distance=%.3fm tolerance=%.3fm",
          vy, raw_distance_m, compensation_m, distance_m,
          params_.move_tolerance_m);
      publishStop();
      phase_ = Phase::EntryPrepareClimb;
      return BT::NodeStatus::RUNNING;
    }
    RCLCPP_INFO(
        node_->get_logger(),
        "梅林预选赛入口1号回2号入口：vy=%.3f raw_distance=%.3fm compensation=%.3fm distance=%.3fm tolerance=%.3fm",
        vy, raw_distance_m, compensation_m, distance_m,
        params_.move_tolerance_m);
    beginMoveRelative(0.0, vy, distance_m,
                      Phase::EntryPrepareClimb, "entry_return_from_stair1");
    return BT::NodeStatus::RUNNING;
  }

  case Phase::EntryMoveRightToStair3:
    // 如果 1 号也没发现目标，从当前 1 号位置一次横移扫到 3 号入口。
    beginMoveRelative(0.0, -std::abs(params_.lateral_probe_speed_mps) *
                               static_cast<double>(params_.field_mirror_sign),
                      params_.entry_probe_right_sweep_distance_m,
                      Phase::EntryDetectStair3, "entry_probe_stair3");
    return BT::NodeStatus::RUNNING;

  case Phase::EntryReturnFromStair3:
  {
    // 3 号入口探测结束后回到中间入口，统一从 grid2 方向入场。
    const double vy = std::abs(params_.lateral_probe_speed_mps) *
                      static_cast<double>(params_.field_mirror_sign);
    const double raw_distance_m = params_.entry_probe_return_distance_m;
    double compensation_m = 0.0;
    const double distance_m =
        MfPreselectionLogicResult::entryReturnToCenterCompensatedDistance(
            raw_distance_m, vy, params_, compensation_m);
    if (distance_m <= params_.move_tolerance_m) {
      RCLCPP_INFO(
          node_->get_logger(),
          "梅林预选赛入口3号回2号入口距离经MCU减速+延迟补偿后已无需横移：vy=%.3f raw_distance=%.3fm compensation=%.3fm distance=%.3fm tolerance=%.3fm",
          vy, raw_distance_m, compensation_m, distance_m,
          params_.move_tolerance_m);
      publishStop();
      phase_ = Phase::EntryPrepareClimb;
      return BT::NodeStatus::RUNNING;
    }
    RCLCPP_INFO(
        node_->get_logger(),
        "梅林预选赛入口3号回2号入口：vy=%.3f raw_distance=%.3fm compensation=%.3fm distance=%.3fm tolerance=%.3fm",
        vy, raw_distance_m, compensation_m, distance_m,
        params_.move_tolerance_m);
    beginMoveRelative(0.0, vy, distance_m,
                      Phase::EntryPrepareClimb, "entry_return_from_stair3");
    return BT::NodeStatus::RUNNING;
  }

  case Phase::EntryReturnToCenterAfterInterruptedPickup:
    return beginEntryReturnToCenterAfterInterruptedPickup();

  case Phase::EntryResumeInterruptedProbeMove:
    return resumeInterruptedEntryMove();

  case Phase::EntryPrepareClimb:
    // 若入口探测阶段已经执行过 ARM_HIGH_RAISE，则保持高抬升姿态上首阶，
    // 避免重复发送普通 ARM_RAISE 造成机构姿态回退或多余等待。
    if (arm_high_raised_) {
      phase_ = Phase::EntryClimb;
      return BT::NodeStatus::RUNNING;
    }
    beginMechanismCommand(clampByte(params_.arm_raise_command_id), "ARM_RAISE",
                          clampByte(params_.arm_raise_done_feedback_id),
                          Phase::EntryClimb, "entry_arm_raise_failed");
    arm_high_side_ = true;
    return BT::NodeStatus::RUNNING;

  case Phase::EntryClimb:
    // 首段从梅林外进入 grid2，复用本文内部台阶原语；完成后 AfterEntry
    // 会把 current_grid 写入黑板并进入中间列推进策略。
    beginStair(StairMode::Climb, Phase::AfterEntry, "entry_climb",
               StairCenterPolicy::EntryGrid2Reference);
    return BT::NodeStatus::RUNNING;

  case Phase::AfterEntry:
    // AfterEntry 是中间列推进的总调度点：每次上/下台阶完成都会回到这里，
    // 再根据当前格号、是否已经夹取、是否进入直出模式决定下一步检测或转场。
    config().blackboard->set("current_grid", current_grid_);
    RCLCPP_INFO(node_->get_logger(),
                "梅林预选赛进入梅林内部决策：current_grid=%d，入场夹取=%s，直出模式=%s，计数=%d/%d",
                current_grid_, entry_pickup_done_ ? "是" : "否",
                direct_exit_mode_ ? "是" : "否", pickup_count_,
                params_.max_pickup_count);
    if (entry_pickup_done_ || direct_exit_mode_) {
      // 直出模式不是“跳过所有安全检查”。它只跳过第 2/3 行的周身搜索，
      // 但前方 R1、R2 KFS 和假 KFS 仍由 RowFront/TransitionObserve/DirectExit
      // 这些守卫阶段处理。
      direct_exit_mode_ = true;
      if (current_grid_ == 2 || current_grid_ == 5 || current_grid_ == 8) {
        RCLCPP_INFO(node_->get_logger(),
                    "梅林预选赛直出模式：grid%d 前方守卫检测后继续中间列推进",
                    current_grid_);
        beginPreparedDetection(DetectMode::RowFront,
                               params_.scan_detect_timeout_s,
                               Phase::RowFrontDetect);
      } else if (current_grid_ == 11) {
        RCLCPP_INFO(node_->get_logger(),
                    "梅林预选赛直出模式到达grid11，进入第四行强制转向收尾");
        phase_ = Phase::Row4ForcedTurn;
      } else if (current_grid_ == 12) {
        RCLCPP_INFO(node_->get_logger(),
                    "梅林预选赛到达grid12，准备先对齐离场yaw再下阶梯");
        phase_ = Phase::FinalExitYawAlign;
      } else {
        beginDirectExitDrive();
      }
    } else if (current_grid_ == 2) {
      RCLCPP_INFO(node_->get_logger(),
                  "梅林预选赛未入场夹取：第一行执行前方检测");
      beginPreparedDetection(DetectMode::RowFront,
                             params_.scan_detect_timeout_s,
                             Phase::RowFrontDetect);
    } else if (current_grid_ == 5 || current_grid_ == 8) {
      // 第 2/3 行先看前方是否被 R1/假 KFS/R2 KFS 占用；如果没有可处理目标，
      // 再左转和背向扫描，降低在狭窄台阶上无意义旋转的概率。
      RCLCPP_INFO(node_->get_logger(),
                  "梅林预选赛未入场夹取：第2/3行先做前方守卫检测");
      beginPreparedDetection(DetectMode::RowFront,
                             params_.scan_detect_timeout_s,
                             Phase::RowFrontDetect);
    } else if (current_grid_ == 11) {
      RCLCPP_INFO(node_->get_logger(),
                  "梅林预选赛到达第四行grid11，进入强制收尾转向");
      phase_ = Phase::Row4ForcedTurn;
    } else {
      phase_ = Phase::TransitionTurn;
    }
    return BT::NodeStatus::RUNNING;

  case Phase::RowScanTurnLeft:
    // 周身扫描分两段：先相对当前朝向左转，再转到背向。这里用 odom_yaw_
    // 叠加参数，而不是写死绝对角，方便现场整体 yaw 标定调整。
    beginTurnYaw(normalizeAngle(odom_yaw_ + params_.row_scan_left_yaw_delta_rad),
                 Phase::RowScanDetectLeft, "row_scan_turn_left");
    return BT::NodeStatus::RUNNING;

  case Phase::RowScanTurnBack:
    beginTurnYaw(normalizeAngle(odom_yaw_ + params_.row_scan_back_yaw_delta_rad),
                 Phase::RowScanDetectBack, "row_scan_turn_back");
    return BT::NodeStatus::RUNNING;

  case Phase::RowAlignExit:
    // 扫描结束后必须重新对齐入口 heading 所代表的出口方向，再进入 grid
    // 间转换；否则台阶原语的 heading hold 会沿错误 yaw 直行。
    beginTurnYaw(entry_heading_yaw_, Phase::TransitionTurn,
                 "row_align_exit");
    return BT::NodeStatus::RUNNING;

  case Phase::FakeAvoidTurn:
  {
    // 假 KFS 是不可夹取目标。第 1/2/3 行前方遇到时，按入口夹取来源选择
    // 向 1 号或 3 号侧绕行；yaw、上/下阶和机构预调都由静态高度表决定。
    if (const auto target_grid =
            MfPreselectionLogicResult::fakeAvoidanceTargetGrid(
                current_grid_, pickup_source_, params_.field_mirror_sign)) {
      fake_avoid_target_grid_ = *target_grid;
    } else {
      return fail("invalid_fake_avoid_target_grid");
    }
    if (!prepareTransitionTo(fake_avoid_target_grid_)) {
      return fail("invalid_fake_avoid_transition");
    }
    const double yaw = transitionYaw(transition_from_grid_, transition_target_grid_,
                                     transition_height_delta_);
    RCLCPP_WARN(node_->get_logger(),
                "梅林预选赛开始假KFS避障格间动作：pickup_source=%s grid%d -> grid%d 高度差=%d 类型=%s 目标yaw=%.3f",
                sourceName(pickup_source_), transition_from_grid_,
                transition_target_grid_, transition_height_delta_,
                transition_height_delta_ > 0 ? "上阶梯" : "下阶梯", yaw);
    beginTurnYaw(yaw, Phase::FakeAvoidArmAdjust, "fake_kfs_avoid_turn");
    return BT::NodeStatus::RUNNING;
  }

  case Phase::FakeAvoidArmAdjust:
    if (transition_high_side_) {
      if (arm_high_raised_) {
        RCLCPP_INFO(node_->get_logger(),
                    "梅林预选赛假KFS避障：机械臂已处于高抬升保持态，直接执行上阶梯");
        phase_ = Phase::FakeAvoidStair;
        return BT::NodeStatus::RUNNING;
      }
      RCLCPP_INFO(node_->get_logger(),
                  "梅林预选赛假KFS避障为低到高，先执行ARM_RAISE后上阶梯");
      beginMechanismCommand(clampByte(params_.arm_raise_command_id),
                            "ARM_RAISE",
                            clampByte(params_.arm_raise_done_feedback_id),
                            Phase::FakeAvoidStair,
                            "fake_avoid_arm_raise_failed");
      arm_high_side_ = true;
      return BT::NodeStatus::RUNNING;
    }
    RCLCPP_INFO(node_->get_logger(),
                "梅林预选赛假KFS避障为高到低，先执行ARM_LOWER后下阶梯");
    beginMechanismCommand(clampByte(params_.arm_lower_command_id),
                          "ARM_LOWER",
                          clampByte(params_.arm_lower_done_feedback_id),
                          Phase::FakeAvoidStair,
                          "fake_avoid_arm_lower_failed");
    arm_high_raised_ = false;
    arm_high_side_ = false;
    return BT::NodeStatus::RUNNING;

  case Phase::FakeAvoidStair:
    beginStair(transition_height_delta_ > 0 ? StairMode::Climb
                                            : StairMode::Descend,
               Phase::FakeAvoidAlignExit,
               transition_height_delta_ > 0 ? "fake_avoid_climb"
                                            : "fake_avoid_descend",
               StairCenterPolicy::FakeAvoidTargetGrid);
    return BT::NodeStatus::RUNNING;

  case Phase::FakeAvoidAlignExit:
    fake_avoid_forward_mode_ = true;
    direct_exit_mode_ = false;
    RCLCPP_INFO(node_->get_logger(),
                "梅林预选赛假KFS避障台阶动作完成，先观察旁列正前方KFS");
    return startFakeAvoidForwardObservation();

  case Phase::FakeAvoidForwardStep:
  {
    fake_avoid_forward_mode_ = true;
    return startFakeAvoidForwardObservation();
  }

  case Phase::TransitionTurn:
    // 预选赛正式路线固定走中间列。这里显式枚举合法下一格，避免因为
    // current_grid 被意外改写而走到未定义的梅林边。
    if (current_grid_ == 2) {
      return startTransitionTo(5);
    } else if (current_grid_ == 5) {
      return startTransitionTo(8);
    } else if (current_grid_ == 8) {
      return startTransitionTo(11);
    } else if (current_grid_ == 11) {
      return startTransitionTo(12);
    } else {
      return fail("no_transition_target");
    }

  case Phase::TransitionArmAdjust:
    // 格间转换前先根据高度差调整机械臂：入口 1/3 侧探测保留
    // ARM_HIGH_RAISE，梅林内部高低台阶观察前统一使用普通 ARM_RAISE/LOWER。
    // 调整完成后仍要做 TransitionObserve，给正前方 R2/R1 目标一个最后处理窗口。
    if (transition_height_delta_ > 0) {
      RCLCPP_INFO(node_->get_logger(),
                  "梅林预选赛格间转换为低到高，先执行ARM_RAISE后观察前方");
      beginMechanismCommand(clampByte(params_.arm_raise_command_id),
                            "ARM_RAISE",
                            clampByte(params_.arm_raise_done_feedback_id),
                            Phase::TransitionObserve,
                            "transition_arm_raise_failed");
      transition_high_side_ = true;
      arm_high_raised_ = false;
      arm_high_side_ = true;
    } else {
      RCLCPP_INFO(node_->get_logger(),
                  "梅林预选赛格间转换为高到低，先执行ARM_LOWER后观察前方");
      beginMechanismCommand(clampByte(params_.arm_lower_command_id),
                            "ARM_LOWER",
                            clampByte(params_.arm_lower_done_feedback_id),
                            Phase::TransitionObserve,
                            "transition_arm_lower_failed");
      transition_high_side_ = false;
      arm_high_raised_ = false;
      arm_high_side_ = false;
    }
    return BT::NodeStatus::RUNNING;

  case Phase::DetectionArmAdjust:
    // 第 2/3 行周身探测和行前方守卫探测也按静态高度表预调机械臂。
    // 低侧探测必须先普通下降 ARM_LOWER，再进入视觉检测窗口。
    if (pending_detection_high_side_) {
      RCLCPP_INFO(node_->get_logger(),
                  "梅林预选赛探测前机械臂预调为高侧：阶段=%s",
                  detectModeText(pending_detection_mode_));
      beginMechanismCommand(clampByte(params_.arm_raise_command_id),
                            "ARM_RAISE",
                            clampByte(params_.arm_raise_done_feedback_id),
                            pending_detection_phase_,
                            "detection_arm_raise_failed");
      arm_high_raised_ = false;
      arm_high_side_ = true;
    } else {
      RCLCPP_INFO(node_->get_logger(),
                  "梅林预选赛探测前机械臂预调为低侧：阶段=%s，发送ARM_LOWER后再检测",
                  detectModeText(pending_detection_mode_));
      beginMechanismCommand(clampByte(params_.arm_lower_command_id),
                            "ARM_LOWER",
                            clampByte(params_.arm_lower_done_feedback_id),
                            pending_detection_phase_,
                            "detection_arm_lower_failed");
      arm_high_raised_ = false;
      arm_high_side_ = false;
    }
    return BT::NodeStatus::RUNNING;

  case Phase::TransitionStair:
    // prepareTransitionTo() 已经确认高度差只能是 +/-1；这里把它映射到
    // 上/下台阶原语。成功后 tickStair() 会通过 continueAfterTransition()
    // 提交 current_grid。
    if (fake_avoid_forward_mode_) {
      const double stair_yaw =
          transitionYaw(transition_from_grid_, transition_target_grid_,
                        transition_height_delta_);
      const double yaw_error = normalizeAngle(stair_yaw - turn_target_yaw_);
      if (std::abs(yaw_error) > params_.turn_tolerance_deg * kDeg2Rad) {
        RCLCPP_INFO(
            node_->get_logger(),
            "梅林预选赛假KFS旁列观察完成，转到台阶执行yaw：grid%d -> grid%d，高度差=%d，target_yaw=%.3f",
            transition_from_grid_, transition_target_grid_,
            transition_height_delta_, stair_yaw);
        beginTurnYaw(stair_yaw, Phase::TransitionStair,
                     "fake_avoid_forward_stair_yaw");
        return BT::NodeStatus::RUNNING;
      }
    }
    RCLCPP_INFO(node_->get_logger(),
                "梅林预选赛开始执行格间台阶动作：grid%d -> grid%d，类型=%s",
                transition_from_grid_, transition_target_grid_,
                transition_height_delta_ > 0 ? "上阶梯" : "下阶梯");
    beginStair(transition_height_delta_ > 0 ? StairMode::Climb
                                            : StairMode::Descend,
               Phase::AfterEntry, "grid_transition",
               StairCenterPolicy::TransitionTargetGrid);
    stair_next_phase_ = Phase::AfterEntry;
    phase_ = Phase::StairPrimitive;
    return BT::NodeStatus::RUNNING;

  case Phase::Row4ForcedTurn:
    // 第四行不再做周身搜索，直接按收尾 yaw 转向；转向后只检查正前方假 KFS，
    // 决定是继续 grid11 -> grid12，还是 180 度转向后直接下阶梯。
    RCLCPP_INFO(node_->get_logger(),
                "梅林预选赛第四行强制转向：target_yaw=%.3f，随后检测正前方假KFS",
                params_.row4_exit_turn_yaw_rad);
    beginTurnYaw(params_.row4_exit_turn_yaw_rad, Phase::Row4DetectFake,
                 "row4_forced_turn");
    return BT::NodeStatus::RUNNING;

  case Phase::Row4FakeTurnBack:
    RCLCPP_WARN(node_->get_logger(),
                "梅林预选赛第四行假KFS分支：执行180度转向后准备下阶梯离场");
    beginTurnYaw(normalizeAngle(odom_yaw_ + kPi),
                 Phase::Row4DirectDescendPrep, "row4_fake_turn_back");
    return BT::NodeStatus::RUNNING;

  case Phase::FinalExitYawAlign:
  {
    const double descend_yaw = normalizeAngle(entry_heading_yaw_ + kPi);
    RCLCPP_INFO(node_->get_logger(),
                "梅林预选赛常规离场：已到grid12台阶上，下阶梯需车头反向，先对齐yaw=%.3f再下阶梯",
                descend_yaw);
    beginTurnYaw(descend_yaw, Phase::Row4DirectDescendPrep,
                 "final_exit_yaw_align");
    return BT::NodeStatus::RUNNING;
  }

  case Phase::Row4DirectDescendPrep:
    // 离场下阶梯前统一把机械臂降下；这既是机构姿态复位，也是给下阶梯
    // GRAB_KFS_DOWN/推杆动作留下明确边界。
    RCLCPP_INFO(node_->get_logger(),
                "梅林预选赛离场前机械臂下降：准备下阶梯离开梅林");
    beginMechanismCommand(clampByte(params_.arm_lower_command_id),
                          "ARM_LOWER",
                          clampByte(params_.arm_lower_done_feedback_id),
                          Phase::Row4DirectDescend,
                          "row4_direct_arm_lower_failed");
    arm_high_raised_ = false;
    arm_high_side_ = false;
    return BT::NodeStatus::RUNNING;

  case Phase::Row4DirectDescend:
    beginStair(StairMode::Descend, Phase::FinalStop, "row4_direct_descend",
               StairCenterPolicy::FinalExitVirtual);
    return BT::NodeStatus::RUNNING;

  case Phase::DirectExitDrive:
    return tickDirectExitDrive();

  case Phase::EntryRetryBackoff:
    return tickEntryRetryBackoff();

  case Phase::RetryPostGrabCenterAlign:
    return beginRetryPostGrabCenterAlign();

  case Phase::FinalStop:
    publishStop();
    writeBlackboardState("final_stop");
    RCLCPP_INFO(node_->get_logger(),
                "梅林预选赛已驶出梅林区域，进入最终停车保持");
    beginZeroHold(0.5, Phase::Done, "final_stop_hold");
    return BT::NodeStatus::RUNNING;

  case Phase::MechanismCommand:
    // 单条机构命令和“双命令并发 ACK”都复用 MechanismCommand phase，
    // 由 command_pair_active_ 区分具体 tick 函数。
    if (command_pair_active_) {
      return tickCommandPair();
    }
    return tickMechanismCommand();
  case Phase::KfsVisualAlign:
    return tickKfsVisualAlign();
  case Phase::KfsSecondArmLower:
    startKfsOdomApproach();
    return BT::NodeStatus::RUNNING;
  case Phase::KfsOdomApproach:
    return tickKfsOdomApproach();
  case Phase::GrabVerify:
    return tickGrabVerify();
  case Phase::MoveRelative:
    return tickMoveRelative();
  case Phase::TurnYaw:
    return tickTurnYaw();
  case Phase::ZeroHold:
    return tickZeroHold();
  case Phase::StairPrimitive:
    return tickStair();
  case Phase::PostGrabCenterAlign:
    return beginPostGrabCenterAlign();
  case Phase::CenterAlign:
    return tickCenterAlign();

  case Phase::Done:
    publishStop();
    if (node_) {
      RCLCPP_INFO(node_->get_logger(),
                  "梅林预选赛流程完成：已夹取=%d/%d，最终grid=%d，进入WaitForever永久静止",
                  pickup_count_, params_.max_pickup_count, current_grid_);
    }
    releaseRuntime();
    config().blackboard->set("mf_preselect_done", true);
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::RUNNING;
}

void MfPreselectionFlowAction::onHalted() {
  if (node_) {
    RCLCPP_WARN(node_->get_logger(),
                "梅林预选赛流程被行为树中断：立即发布零速并释放运行接口");
  }
  // halt 可能来自外层 BT 停止、树切换或节点析构。这里不尝试补偿推杆动作，
  // 只保证运动输出归零并让异步回调通过 generation token 自然失效。
  releaseRuntime();
}

bool MfPreselectionFlowAction::setupRuntime() {
  // decision_node 在黑板里注入 ROS node、MF 预选赛参数和通用台阶参数。
  // 缺任一项都说明当前 BT 装配不完整，不能以默认值猜测实车行为。
  if (!config().blackboard || !config().blackboard->get("node", node_) ||
      !node_) {
    writeDecisionFailure(config().blackboard, "MfPreselectionFlow",
                         "运行上下文缺失：blackboard 或 node 不可用");
    return false;
  }
  if (!config().blackboard->get("mf_preselection_params", params_)) {
    RCLCPP_ERROR(node_->get_logger(), "梅林预选赛: 黑板缺少 mf_preselection_params");
    writeDecisionFailure(config().blackboard, "MfPreselectionFlow",
                         "黑板缺少 mf_preselection_params");
    return false;
  }
  if (!config().blackboard->get("stair_params", stair_params_)) {
    RCLCPP_ERROR(node_->get_logger(), "梅林预选赛: 黑板缺少 stair_params");
    writeDecisionFailure(config().blackboard, "MfPreselectionFlow",
                         "黑板缺少 stair_params");
    return false;
  }
  if (!config().blackboard->get("mf_center_params", center_params_)) {
    RCLCPP_ERROR(node_->get_logger(), "梅林预选赛: 黑板缺少 mf_center_params");
    writeDecisionFailure(config().blackboard, "MfPreselectionFlow",
                         "黑板缺少 mf_center_params");
    return false;
  }
  normalizeParams();

  cmd_pub_ =
      node_->create_publisher<TwistMsg>(params_.cmd_vel_topic, rclcpp::QoS(10));
  center_cmd_pub_ = node_->create_publisher<TwistMsg>(
      center_params_.cmd_vel_topic, rclcpp::QoS(10));
  send_client_ = node_->create_client<SendCommandSrv>(params_.send_command_service);
  // command_feedback 同时承担两类信息：
  // 1. 台阶激光事件计数，tickStair() 只看相对 baseline 是否增长；
  // 2. 带 seq 的机构完成反馈，用于 ARM_RAISE/LOWER 等需要等待 done 的命令。
  feedback_sub_ = node_->create_subscription<FeedbackMsg>(
      params_.feedback_topic, rclcpp::QoS(32).reliable(),
      [this](const FeedbackMsg::SharedPtr msg) {
        if (!msg) {
          return;
        }
        if (msg->feedback_id ==
            static_cast<uint8_t>(rc26_serial::FeedbackID::FRONT_LASER_HEIGHT_JUMP)) {
          front_first_event_count_.fetch_add(1, std::memory_order_relaxed);
        } else if (msg->feedback_id ==
                   static_cast<uint8_t>(rc26_serial::FeedbackID::FRONT_SECOND_LASER_HEIGHT_JUMP)) {
          front_second_event_count_.fetch_add(1, std::memory_order_relaxed);
        } else if (msg->feedback_id ==
                   static_cast<uint8_t>(rc26_serial::FeedbackID::REAR_LASER_HEIGHT_JUMP)) {
          rear_event_count_.fetch_add(1, std::memory_order_relaxed);
        }
        const int expected_feedback = command_done_feedback_id_;
        const int expected_seq = command_seq_.load(std::memory_order_relaxed);
        if (expected_feedback >= 0 && expected_seq >= 0 &&
            msg->feedback_id == static_cast<uint8_t>(expected_feedback) &&
            msg->seq == static_cast<uint8_t>(expected_seq)) {
          // 只接受当前命令 seq 对应的完成反馈，避免上一条命令的延迟反馈误推进
          // 当前状态机。
          const bool already_done =
              command_done_seen_.exchange(true, std::memory_order_relaxed);
          if (!already_done && node_) {
            RCLCPP_INFO(node_->get_logger(),
                        "梅林预选赛收到机构完成反馈：feedback=0x%02X seq=%d",
                        static_cast<unsigned int>(msg->feedback_id),
                        static_cast<int>(msg->seq));
          }
        }
      });
  if (!params_.odom_topic.empty()) {
    odom_sub_ = node_->create_subscription<OdomMsg>(
        params_.odom_topic, rclcpp::QoS(rclcpp::KeepLast(10)),
        [this](const OdomMsg::SharedPtr msg) { handleOdom(msg); });
  }
  if (!center_params_.odom_topic.empty()) {
    center_odom_sub_ = node_->create_subscription<OdomMsg>(
        center_params_.odom_topic, rclcpp::QoS(rclcpp::KeepLast(10)),
        [this](const OdomMsg::SharedPtr msg) { handleCenterOdom(msg); });
  }

  has_odom_ = false;
  has_center_odom_ = false;
  // generation 每次启动或释放都会递增；异步 service 回调先检查 token，
  // 防止流程 halt 后旧 future 回调写回新一轮状态。
  command_generation_.fetch_add(1, std::memory_order_relaxed);
  config().blackboard->set("mf_preselect_pickup_count", 0);
  config().blackboard->set("mf_preselect_pickup_source", std::string("无"));
  config().blackboard->set("mf_preselect_done", false);
  RCLCPP_INFO(node_->get_logger(),
              "梅林预选赛运行接口就绪：cmd_vel=%s odom=%s center_cmd_vel=%s center_odom=%s command_service=%s feedback=%s entry_interrupt_offset<=%dpx dynamic_comp=%s fx=%.1f latency=%.3fs extra=[%d,%d]px stop_settle=%s acc=%.3fm/s^2 margin=%.3fs max_wait=%.3fs kfs_align_target_line_offset=%dpx kfs_align_speed=[%.3f, %.3f]m/s timeout_pickup<=%dpx kfs_approach_odom_kp=%.3f approach_tol=%.3fm approach_speed=%.3fm/s approach_min=%.3fm/s arm_reach=%.3fm approach_timeout=%.2fs mono_fallback=%s mono_size=[%.3f,%.3f]m mono_fx_fy=[%.1f,%.1f]px mono_min_bbox=%dpx mono_max_delta=%.3fm depth_roi=%d depth_min_valid=%d bbox_sample_ratios=%zu bbox_min_success=%d",
              params_.cmd_vel_topic.c_str(), params_.odom_topic.c_str(),
              center_params_.cmd_vel_topic.c_str(),
              center_params_.odom_topic.c_str(),
              params_.send_command_service.c_str(), params_.feedback_topic.c_str(),
              params_.entry_interrupt_max_offset_px,
              params_.entry_interrupt_dynamic_comp_enable ? "开" : "关",
              params_.entry_interrupt_fx_px,
              params_.entry_interrupt_latency_s,
              params_.entry_interrupt_extra_px_min,
              params_.entry_interrupt_extra_px_max,
              params_.entry_mcu_stop_settle_enable ? "开" : "关",
              params_.entry_mcu_vy_acc_mps2,
              params_.entry_mcu_stop_margin_s,
              params_.entry_mcu_stop_max_wait_s,
              params_.kfs_align_target_line_offset_px,
              params_.kfs_align_min_speed_mps, params_.kfs_align_max_speed_mps,
              params_.kfs_align_timeout_pickup_tolerance_px,
              params_.kfs_odom_xy_kp, params_.kfs_approach_odom_tolerance_m,
              params_.kfs_approach_speed_mps, params_.kfs_approach_min_speed_mps,
              params_.kfs_grab_distance_m,
              params_.kfs_approach_timeout_s,
              params_.kfs_mono_distance_fallback_enable ? "开" : "关",
              params_.kfs_mono_target_width_m,
              params_.kfs_mono_target_height_m, params_.kfs_mono_fx_px,
              params_.kfs_mono_fy_px, params_.kfs_mono_min_bbox_px,
              params_.kfs_mono_max_delta_from_locked_m,
              params_.kfs_depth_roi_size,
              params_.kfs_depth_min_valid_count,
              params_.kfs_depth_bbox_sample_ratios.size(),
              params_.kfs_depth_bbox_min_success_count);
  return true;
}

bool MfPreselectionFlowAction::setupVision() {
  // 本节点直接创建 VisionInferenceManager，而不是订阅其它公开检测 topic。
  // 这样可在一个 BT 节点内按最新帧快照同时使用彩色、深度和 detection 序号。
  try {
    params_.vision_config_file = resolveVisionConfig(params_.vision_config_file);
    auto config = rc26_vision::ProfileLoader::loadFromYaml(params_.vision_config_file);
    rc26_vision::ProfileLoader::validate(config);
    if (config.profiles.find(params_.model_id) == config.profiles.end()) {
      RCLCPP_ERROR(node_->get_logger(), "梅林预选赛 KFS profile 不存在: %s",
                   params_.model_id.c_str());
      return false;
    }
    vision_ = std::make_shared<rc26_vision::VisionInferenceManager>(*node_);
    vision_->loadConfig(config);
    vision_->selectModel(params_.model_id);
    if (!vision_->start()) {
      RCLCPP_ERROR(node_->get_logger(), "梅林预选赛 KFS 视觉启动失败");
      vision_.reset();
      return false;
    }
    RCLCPP_INFO(node_->get_logger(),
                "梅林预选赛 KFS 视觉已启动：config=%s model=%s R2前缀数=%zu R1标签数=%zu 假KFS前缀数=%zu 通用深度=[%.2f, %.2f]m 入口深度=[%.2f, %.2f]m 入口打断offset<=%dpx dynamic_comp=%s 识别框中线目标偏置=%dpx 横移视觉速度=[%.3f, %.3f]m/s 超时补夹offset<=%dpx 前向odom速度=%.3fm/s 臂长=%.3fm 超时=%.2fs mono_fallback=%s mono_size=[%.3f,%.3f]m mono_fx_fy=[%.1f,%.1f]px mono_max_delta=%.3fm depth_roi=%d depth_min_valid=%d bbox_sample_ratios=%zu bbox_min_success=%d",
                params_.vision_config_file.c_str(), params_.model_id.c_str(),
                params_.r2_target_label_prefixes.size(),
                params_.r1_blocking_labels.size(),
                params_.fake_label_prefixes.size(), params_.depth_min_m,
                params_.depth_max_m, params_.entry_depth_min_m,
                params_.entry_depth_max_m, params_.entry_interrupt_max_offset_px,
                params_.entry_interrupt_dynamic_comp_enable ? "开" : "关",
                params_.kfs_align_target_line_offset_px,
                params_.kfs_align_min_speed_mps,
                params_.kfs_align_max_speed_mps,
                params_.kfs_align_timeout_pickup_tolerance_px,
                params_.kfs_approach_speed_mps,
                params_.kfs_grab_distance_m, params_.kfs_approach_timeout_s,
                params_.kfs_mono_distance_fallback_enable ? "开" : "关",
                params_.kfs_mono_target_width_m,
                params_.kfs_mono_target_height_m, params_.kfs_mono_fx_px,
                params_.kfs_mono_fy_px,
                params_.kfs_mono_max_delta_from_locked_m,
                params_.kfs_depth_roi_size,
                params_.kfs_depth_min_valid_count,
                params_.kfs_depth_bbox_sample_ratios.size(),
                params_.kfs_depth_bbox_min_success_count);
    return true;
  } catch (const std::exception &e) {
    RCLCPP_ERROR(node_->get_logger(), "梅林预选赛 KFS 视觉初始化异常: %s",
                 e.what());
    vision_.reset();
    return false;
  }
}

void MfPreselectionFlowAction::releaseRuntime() {
  // releaseRuntime() 必须可重入：onHalted()、fail()、析构和 Done 分支都可能调用。
  // 先停车再释放 publisher/client/subscription，保证最后一个可发布动作是零速。
  publishStop();
  publishCenterStop();
  if (vision_) {
    vision_->stop();
    vision_.reset();
  }
  command_generation_.fetch_add(1, std::memory_order_relaxed);
  feedback_sub_.reset();
  center_odom_sub_.reset();
  odom_sub_.reset();
  send_client_.reset();
  center_cmd_pub_.reset();
  cmd_pub_.reset();
  node_ = nullptr;
  phase_ = Phase::Done;
  detection_active_ = false;
  direct_exit_move_active_ = false;
  fake_avoid_forward_mode_ = false;
  center_target_ready_ = false;
  pending_grab_commit_ = false;
  pending_grab_source_ = MfPreselectionPickupSource::None;
  pending_grab_target_.reset();
  ignored_r2_targets_.clear();
  grab_success_direct_exit_ = false;
  post_grab_center_next_phase_ = Phase::Done;
  clearGrabRetryContext();
  clearKfsVisualPickup();
}

void MfPreselectionFlowAction::normalizeParams() {
  // 参数规范化集中在运行前做一次。这里不把异常配置直接判失败，而是把符号、
  // 空列表、最小超时等修正到可执行范围，便于现场 YAML 小幅调参。
  params_.vision_config_file = resolveVisionConfig(params_.vision_config_file);
  params_.r2_target_label_prefixes =
      sanitized(std::move(params_.r2_target_label_prefixes));
  params_.r2_target_labels = sanitized(std::move(params_.r2_target_labels));
  params_.r1_blocking_labels = sanitized(std::move(params_.r1_blocking_labels));
  params_.r1_blocking_label_prefixes =
      sanitized(std::move(params_.r1_blocking_label_prefixes));
  params_.r1_kfs_min_score =
      std::clamp(std::isfinite(params_.r1_kfs_min_score)
                     ? params_.r1_kfs_min_score
                     : 0.50,
                 0.0, 1.0);
  params_.fake_label_prefixes = sanitized(std::move(params_.fake_label_prefixes));
  params_.fake_labels = sanitized(std::move(params_.fake_labels));
  params_.depth_min_m = std::max(0.0, params_.depth_min_m);
  params_.depth_max_m = std::max(params_.depth_min_m, params_.depth_max_m);
  params_.entry_depth_min_m = std::max(0.0, params_.entry_depth_min_m);
  params_.entry_depth_max_m =
      std::max(params_.entry_depth_min_m, params_.entry_depth_max_m);
  params_.detect_seen_stable_frames =
      std::max(1, params_.detect_seen_stable_frames);
  params_.detect_lost_stable_frames =
      std::max(1, params_.detect_lost_stable_frames);
  params_.entry_detect_timeout_s =
      std::max(kMinTimeoutS, params_.entry_detect_timeout_s);
  params_.scan_detect_timeout_s =
      std::max(kMinTimeoutS, params_.scan_detect_timeout_s);
  params_.entry_interrupt_max_offset_px =
      std::max(0, params_.entry_interrupt_max_offset_px);
  params_.entry_interrupt_latency_s =
      std::max(0.0, std::isfinite(params_.entry_interrupt_latency_s)
                        ? params_.entry_interrupt_latency_s
                        : 0.0);
  if (!std::isfinite(params_.entry_interrupt_fx_px) ||
      params_.entry_interrupt_fx_px < 0.0) {
    params_.entry_interrupt_fx_px = 0.0;
  }
  params_.entry_interrupt_extra_px_min =
      std::max(0, params_.entry_interrupt_extra_px_min);
  params_.entry_interrupt_extra_px_max =
      std::max(params_.entry_interrupt_extra_px_min,
               params_.entry_interrupt_extra_px_max);
  if (!std::isfinite(params_.entry_mcu_vy_acc_mps2) ||
      params_.entry_mcu_vy_acc_mps2 < 0.0) {
    params_.entry_mcu_vy_acc_mps2 = 0.0;
  }
  params_.entry_mcu_stop_margin_s =
      std::max(0.0, std::isfinite(params_.entry_mcu_stop_margin_s)
                        ? params_.entry_mcu_stop_margin_s
                        : 0.0);
  params_.entry_mcu_stop_max_wait_s =
      std::max(0.0, std::isfinite(params_.entry_mcu_stop_max_wait_s)
                        ? params_.entry_mcu_stop_max_wait_s
                        : 0.0);
  params_.kfs_align_tolerance_px =
      std::max(0, params_.kfs_align_tolerance_px);
  params_.kfs_align_target_line_offset_px =
      std::clamp(params_.kfs_align_target_line_offset_px, -10000, 10000);
  params_.kfs_align_stable_frames =
      std::max(1, params_.kfs_align_stable_frames);
  params_.kfs_align_kp = std::max(0.0, params_.kfs_align_kp);
  params_.kfs_align_min_speed_mps =
      std::max(0.0, std::abs(params_.kfs_align_min_speed_mps));
  params_.kfs_align_max_speed_mps =
      std::max(params_.kfs_align_min_speed_mps,
               std::abs(params_.kfs_align_max_speed_mps));
  if (!std::isfinite(params_.kfs_align_timeout_s) ||
      params_.kfs_align_timeout_s <= 0.0) {
    params_.kfs_align_timeout_s = kMinTimeoutS;
  }
  params_.kfs_align_timeout_pickup_tolerance_px =
      std::max(params_.kfs_align_tolerance_px,
               params_.kfs_align_timeout_pickup_tolerance_px);
  params_.kfs_align_heading_gate_deg =
      std::max(0.0, finiteAbsOr(params_.kfs_align_heading_gate_deg, 8.0));
  params_.kfs_lost_stop_frames =
      std::max(1, params_.kfs_lost_stop_frames);
  if (!std::isfinite(params_.kfs_approach_speed_mps) ||
      params_.kfs_approach_speed_mps < 0.0) {
    params_.kfs_approach_speed_mps = 0.0;
  }
  params_.kfs_approach_x_sign =
      params_.kfs_approach_x_sign < 0 ? -1 : 1;
  MfPreselectionLogicResult::normalizeKfsOdomParams(params_);
  if (!std::isfinite(params_.kfs_approach_timeout_s) ||
      params_.kfs_approach_timeout_s <= 0.0) {
    params_.kfs_approach_timeout_s = kMinTimeoutS;
  }
  if (!std::isfinite(params_.kfs_grab_distance_m) ||
      params_.kfs_grab_distance_m < 0.0) {
    params_.kfs_grab_distance_m = 0.0;
  }
  params_.kfs_mono_target_width_m =
      std::max(0.0, std::isfinite(params_.kfs_mono_target_width_m)
                        ? params_.kfs_mono_target_width_m
                        : 0.0);
  params_.kfs_mono_target_height_m =
      std::max(0.0, std::isfinite(params_.kfs_mono_target_height_m)
                        ? params_.kfs_mono_target_height_m
                        : 0.0);
  params_.kfs_mono_fx_px =
      std::max(0.0, std::isfinite(params_.kfs_mono_fx_px)
                        ? params_.kfs_mono_fx_px
                        : 0.0);
  params_.kfs_mono_fy_px =
      std::max(0.0, std::isfinite(params_.kfs_mono_fy_px)
                        ? params_.kfs_mono_fy_px
                        : 0.0);
  params_.kfs_mono_min_bbox_px =
      std::max(1, params_.kfs_mono_min_bbox_px);
  params_.kfs_mono_max_delta_from_locked_m =
      std::max(0.0, std::isfinite(params_.kfs_mono_max_delta_from_locked_m)
                        ? params_.kfs_mono_max_delta_from_locked_m
                        : 0.0);
  params_.kfs_depth_roi_size = std::max(1, params_.kfs_depth_roi_size);
  params_.kfs_depth_min_valid_count =
      std::max(1, params_.kfs_depth_min_valid_count);
  std::vector<double> sanitized_depth_ratios;
  sanitized_depth_ratios.reserve(params_.kfs_depth_bbox_sample_ratios.size());
  for (const double ratio : params_.kfs_depth_bbox_sample_ratios) {
    if (std::isfinite(ratio)) {
      sanitized_depth_ratios.push_back(std::clamp(ratio, 0.0, 1.0));
    }
  }
  if (sanitized_depth_ratios.empty()) {
    sanitized_depth_ratios = {0.25, 0.50, 0.75};
  }
  std::sort(sanitized_depth_ratios.begin(), sanitized_depth_ratios.end());
  sanitized_depth_ratios.erase(
      std::unique(sanitized_depth_ratios.begin(), sanitized_depth_ratios.end()),
      sanitized_depth_ratios.end());
  params_.kfs_depth_bbox_sample_ratios = std::move(sanitized_depth_ratios);
  const int max_bbox_success_count = static_cast<int>(
      params_.kfs_depth_bbox_sample_ratios.size() *
      params_.kfs_depth_bbox_sample_ratios.size());
  params_.kfs_depth_bbox_min_success_count = std::clamp(
      params_.kfs_depth_bbox_min_success_count, 1, max_bbox_success_count);
  params_.max_pickup_count = std::max(0, params_.max_pickup_count);
  params_.grab_settle_s = std::max(0.0, params_.grab_settle_s);
  params_.grab_verify_timeout_s =
      std::max(kMinTimeoutS, params_.grab_verify_timeout_s);
  params_.grab_verify_lost_stable_frames =
      std::max(1, params_.grab_verify_lost_stable_frames);
  params_.grab_verify_iou_threshold =
      std::clamp(params_.grab_verify_iou_threshold, 0.0, 1.0);
  params_.command_timeout_s = std::max(kMinTimeoutS, params_.command_timeout_s);
  params_.entry_probe_left_distance_m =
      std::max(0.0, std::abs(params_.entry_probe_left_distance_m));
  params_.entry_probe_right_sweep_distance_m =
      std::max(0.0, std::abs(params_.entry_probe_right_sweep_distance_m));
  params_.entry_probe_return_distance_m =
      std::max(0.0, std::abs(params_.entry_probe_return_distance_m));
  params_.lateral_probe_speed_mps =
      std::max(kMinSpeed, std::abs(params_.lateral_probe_speed_mps));
  params_.move_tolerance_m = std::max(0.0, std::abs(params_.move_tolerance_m));
  params_.move_timeout_s = std::max(kMinTimeoutS, params_.move_timeout_s);
  params_.direct_exit_drive_distance_m =
      std::max(0.0, std::abs(params_.direct_exit_drive_distance_m));
  params_.direct_exit_drive_speed_mps =
      std::max(kMinSpeed, std::abs(params_.direct_exit_drive_speed_mps));
  params_.turn_kp = std::max(0.0, params_.turn_kp);
  params_.turn_max_speed_radps =
      std::max(0.0, std::abs(params_.turn_max_speed_radps));
  params_.turn_tolerance_deg = std::max(0.0, params_.turn_tolerance_deg);
  params_.turn_stable_ticks = std::max(1, params_.turn_stable_ticks);
  params_.turn_timeout_s = std::max(kMinTimeoutS, params_.turn_timeout_s);
  params_.odom_timeout_s = std::max(kMinTimeoutS, params_.odom_timeout_s);
  params_.heading_kp = std::max(0.0, params_.heading_kp);
  params_.heading_max_speed_radps =
      std::max(0.0, std::abs(params_.heading_max_speed_radps));
  params_.path_r1_lost_wait_timeout_s =
      std::max(0.0, params_.path_r1_lost_wait_timeout_s);
  params_.final_exit_center_offset_m =
      std::max(0.0, finiteAbsOr(params_.final_exit_center_offset_m, 1.2));

  normalizeStairParams(stair_params_);

  center_params_.grid_step_m =
      std::max(kMinSpeed, finiteAbsOr(center_params_.grid_step_m, 1.2));
  center_params_.entry_forward_offset_m = std::max(
      0.0, std::isfinite(center_params_.entry_forward_offset_m)
               ? center_params_.entry_forward_offset_m
               : 0.25);
  center_params_.entry_forward_speed_mps = std::max(
      kMinSpeed, finiteAbsOr(center_params_.entry_forward_speed_mps, 0.04));
  center_params_.xy_kp =
      std::max(0.0, std::isfinite(center_params_.xy_kp)
                        ? center_params_.xy_kp
                        : 0.8);
  center_params_.min_speed_mps =
      std::max(0.0, finiteAbsOr(center_params_.min_speed_mps, 0.010));
  center_params_.max_speed_mps =
      std::max(center_params_.min_speed_mps,
               finiteAbsOr(center_params_.max_speed_mps, 0.050));
  center_params_.xy_tolerance_m =
      std::max(kMinSpeed, finiteAbsOr(center_params_.xy_tolerance_m, 0.035));
  center_params_.yaw_kp =
      std::max(0.0, std::isfinite(center_params_.yaw_kp)
                        ? center_params_.yaw_kp
                        : 1.2);
  center_params_.yaw_max_speed_radps =
      std::max(0.0, finiteAbsOr(center_params_.yaw_max_speed_radps, 0.30));
  center_params_.yaw_tolerance_deg =
      std::max(0.0, finiteAbsOr(center_params_.yaw_tolerance_deg, 3.0));
  center_params_.stable_ticks = std::max(1, center_params_.stable_ticks);
  center_params_.odom_timeout_s =
      std::max(kMinTimeoutS, finiteAbsOr(center_params_.odom_timeout_s, 0.5));
  center_params_.align_timeout_s =
      std::max(kMinTimeoutS, finiteAbsOr(center_params_.align_timeout_s, 8.0));
}

BT::NodeStatus MfPreselectionFlowAction::fail(const std::string &reason) {
  // 失败路径统一写黑板 mf_preselect_error，方便外层日志或后续诊断知道卡在哪个
  // 语义阶段，而不是只能从最后一条 ROS 日志倒推。
  std::string detail = translateMfFailureReason(reason);
  detail += "，阶段=" + std::string(phaseText(phase_));
  detail += "，当前格=" + std::to_string(current_grid_);
  if (command_sent_ ||
      command_response_seen_.load(std::memory_order_relaxed) ||
      !command_label_.empty()) {
    detail += "，命令=" + command_label_;
    detail += "(0x" + byteHex(command_id_) + ")";
    detail += "，seq=" + seqText(command_seq_.load(std::memory_order_relaxed));
    detail += (command_accepted_.load(std::memory_order_relaxed)
                   ? " 已接受=是"
                   : " 已接受=否");
    if (command_done_feedback_id_ >= 0) {
      detail += " 完成反馈=0x" +
                byteHex(static_cast<uint8_t>(command_done_feedback_id_));
      detail += (command_done_seen_.load(std::memory_order_relaxed)
                     ? " 已收到完成反馈=是"
                     : " 已收到完成反馈=否");
    }
  }
  if (command_pair_active_) {
    detail += "，并发命令=" + command_pair_[0].label + "(0x" +
              byteHex(command_pair_[0].command_id) + ",seq=" +
              seqText(command_pair_[0].seq.load(std::memory_order_relaxed)) +
              ",已接受=" +
              (command_pair_[0].accepted.load(std::memory_order_relaxed)
                   ? "是"
                   : "否") +
              ")+" + command_pair_[1].label + "(0x" +
              byteHex(command_pair_[1].command_id) + ",seq=" +
              seqText(command_pair_[1].seq.load(std::memory_order_relaxed)) +
              ",已接受=" +
              (command_pair_[1].accepted.load(std::memory_order_relaxed)
                   ? "是"
                   : "否") +
              ")";
  }
  if (node_) {
    RCLCPP_ERROR(node_->get_logger(), "梅林预选赛失败: %s", detail.c_str());
  }
  if (config().blackboard) {
    config().blackboard->set("mf_preselect_error", detail);
    writeDecisionFailure(config().blackboard, "MfPreselectionFlow", detail);
  }
  releaseRuntime();
  return BT::NodeStatus::FAILURE;
}

void MfPreselectionFlowAction::publishStop() { publishTwist(0.0, 0.0, 0.0); }

void MfPreselectionFlowAction::publishCenterStop() {
  publishCenterTwist(0.0, 0.0, 0.0);
}

void MfPreselectionFlowAction::publishCenterTwist(double vx, double vy,
                                                  double wz) {
  if (!center_cmd_pub_) {
    publishTwist(vx, vy, wz);
    return;
  }
  TwistMsg msg;
  msg.linear.x = vx;
  msg.linear.y = vy;
  msg.angular.z = wz;
  center_cmd_pub_->publish(msg);
  if (node_) {
    last_cmd_publish_ = node_->now();
    has_last_cmd_publish_ = true;
  }
}

void MfPreselectionFlowAction::publishTwist(double vx, double vy, double wz) {
  // 本树内所有相对移动、转向和台阶动作都走同一个 cmd_vel publisher。
  // 这也意味着外部 launch 必须保证同一时刻没有其它运动命令权威。
  if (!cmd_pub_) {
    return;
  }
  TwistMsg msg;
  msg.linear.x = vx;
  msg.linear.y = vy;
  msg.angular.z = wz;
  cmd_pub_->publish(msg);
  if (node_) {
    last_cmd_publish_ = node_->now();
    has_last_cmd_publish_ = true;
  }
}

bool MfPreselectionFlowAction::odomReady() const {
  // 使用 steady_clock 计算 odom 新鲜度，避免 ROS time 在仿真/回放场景跳变时
  // 影响“是否还能闭环发布速度”的安全判断。
  if (!has_odom_) {
    return false;
  }
  const auto age_s = std::chrono::duration<double>(
                         std::chrono::steady_clock::now() - last_odom_tp_)
                         .count();
  return age_s <= params_.odom_timeout_s;
}

bool MfPreselectionFlowAction::centerOdomReady() const {
  if (!has_center_odom_) {
    return false;
  }
  const auto age_s =
      std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                    last_center_odom_tp_)
          .count();
  return age_s <= center_params_.odom_timeout_s;
}

void MfPreselectionFlowAction::handleOdom(const OdomMsg::SharedPtr msg) {
  // 预选赛只需要 odom 平面位姿：相对移动用 x/y 差，转向和直行 heading hold 用 yaw。
  if (!msg) {
    return;
  }
  odom_x_ = msg->pose.pose.position.x;
  odom_y_ = msg->pose.pose.position.y;
  odom_yaw_ = yawFromQuaternion(msg->pose.pose.orientation);
  has_odom_ = true;
  last_odom_tp_ = std::chrono::steady_clock::now();
}

void MfPreselectionFlowAction::handleCenterOdom(const OdomMsg::SharedPtr msg) {
  if (!msg) {
    return;
  }
  center_odom_x_ = msg->pose.pose.position.x;
  center_odom_y_ = msg->pose.pose.position.y;
  center_odom_yaw_ = yawFromQuaternion(msg->pose.pose.orientation);
  has_center_odom_ = true;
  last_center_odom_tp_ = std::chrono::steady_clock::now();
}

double MfPreselectionFlowAction::yawFromQuaternion(
    const geometry_msgs::msg::Quaternion &q) {
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

double MfPreselectionFlowAction::normalizeAngle(double angle_rad) {
  while (angle_rad > kPi) {
    angle_rad -= 2.0 * kPi;
  }
  while (angle_rad < -kPi) {
    angle_rad += 2.0 * kPi;
  }
  return angle_rad;
}

double MfPreselectionFlowAction::headingAngularZ(double target_yaw_rad) const {
  // 直线段的 heading hold 是轻量 P 控制：没有新鲜 odom 时宁可不给角速度，
  // 让上层 tick 继续用 guard/timeout 控制，而不是基于旧姿态纠偏。
  if (!odomReady()) {
    return 0.0;
  }
  const double error = normalizeAngle(target_yaw_rad - odom_yaw_);
  const double raw = params_.heading_kp * error;
  return std::clamp(raw, -params_.heading_max_speed_radps,
                    params_.heading_max_speed_radps);
}

std::optional<MfPreselectionTargetSnapshot>
MfPreselectionFlowAction::findR2Target() {
  // R2 可夹取目标受 max_pickup_count 限制；达到上限后视觉仍可运行，
  // 但不会再把 T_* 目标转成夹取动作。
  if (!canPickup()) {
    return std::nullopt;
  }
  return findTarget(params_.r2_target_labels, params_.r2_target_label_prefixes,
                    true);
}

std::optional<MfPreselectionTargetSnapshot>
MfPreselectionFlowAction::findR1BlockingTarget() {
  return findTarget(params_.r1_blocking_labels,
                    params_.r1_blocking_label_prefixes, false, true);
}

std::optional<MfPreselectionTargetSnapshot>
MfPreselectionFlowAction::findFakeTarget() {
  return findTarget(params_.fake_labels, params_.fake_label_prefixes, false);
}

std::optional<MfPreselectionTargetSnapshot>
MfPreselectionFlowAction::findTarget(const std::vector<std::string> &exact,
                                     const std::vector<std::string> &prefixes,
                                     bool skip_ignored_r2,
                                     bool filter_r1_kfs_score) {
  // 目标读取是“快照式”的：同一帧里的 detection 和 depth 一起使用，
  // 避免检测框来自新帧、深度来自旧帧导致距离门限失真。
  if (!vision_ || !vision_->isRunning()) {
    return std::nullopt;
  }
  rc26_vision::VisionInferenceManager::FrameSnapshot snapshot;
  if (!vision_->getLatestFrameSnapshot(snapshot) || !snapshot.has_display ||
      !snapshot.has_depth) {
    return std::nullopt;
  }

  const rc26_vision::Detection *best = nullptr;
  for (const auto &det : snapshot.detections) {
    const std::string name = rc26_vision::visualTargetLabel(det);
    if (!MfPreselectionLogicResult::labelMatches(name, exact, prefixes)) {
      continue;
    }
    if (filter_r1_kfs_score &&
        !MfPreselectionLogicResult::r1KfsScoreAccepted(
            name, det.score, params_.r1_kfs_min_score)) {
      if (node_ && snapshot.display_sequence !=
                       r1_kfs_low_score_last_logged_sequence_) {
        r1_kfs_low_score_last_logged_sequence_ = snapshot.display_sequence;
        RCLCPP_INFO(
            node_->get_logger(),
            "梅林预选赛过滤低置信度R1_KFS：score=%.3f threshold=%.3f seq=%ld bbox=[%.1f %.1f %.1f %.1f]",
            det.score, params_.r1_kfs_min_score,
            static_cast<long>(snapshot.display_sequence), det.x1, det.y1,
            det.x2, det.y2);
      }
      continue;
    }
    const MfPreselectionTargetSnapshot candidate =
        rc26_vision::makeVisualTargetSnapshot(det, snapshot.display_sequence);
    if (skip_ignored_r2 &&
        MfPreselectionLogicResult::isIgnoredTarget(
            candidate, ignored_r2_targets_, params_.grab_verify_iou_threshold)) {
      continue;
    }
    // 同类候选里选 score 最高的框。这里不做目标锁定，预选赛检测阶段只需要
    // 判断“当前方向是否存在可处理目标”，不是视觉伺服对准。
    if (!best || det.score > best->score) {
      best = &det;
    }
  }
  if (!best) {
    return std::nullopt;
  }

  const int cx = static_cast<int>((best->x1 + best->x2) * 0.5F);
  const int cy = static_cast<int>((best->y1 + best->y2) * 0.5F);
  rc26_vision::DepthRoiSamplerConfig depth_config;
  depth_config.roi_size = params_.kfs_depth_roi_size;
  depth_config.min_valid_count = params_.kfs_depth_min_valid_count;
  depth_config.min_depth_m = params_.depth_min_m;
  depth_config.max_depth_m = params_.depth_max_m;
  // 深度门限是检测有效性的第二道过滤：模型看到标签但 ROI 深度不可信时，
  // 当前 tick 按“未看到目标”处理，等待后续稳定帧。
  const auto sampled =
      rc26_vision::sampleMedianDepth(snapshot.depth, cx, cy, depth_config);
  if (!sampled.has_value()) {
    return std::nullopt;
  }

  MfPreselectionTargetSnapshot obs =
      rc26_vision::makeVisualTargetSnapshot(*best, snapshot.display_sequence);
  obs.distance_m = *sampled;
  return obs;
}

std::optional<int64_t> MfPreselectionFlowAction::latestVisionSequence() const {
  // 检测阶段用 display_sequence 判断“是否来了新视觉帧”。如果没有新帧，
  // 不增加 lost 计数，避免相机/推理帧率低时过早认为目标已经消失。
  if (!vision_ || !vision_->isRunning()) {
    return std::nullopt;
  }
  rc26_vision::VisionInferenceManager::FrameSnapshot snapshot;
  if (!vision_->getLatestFrameSnapshot(snapshot) || !snapshot.has_display ||
      snapshot.display_sequence <= 0) {
    return std::nullopt;
  }
  return snapshot.display_sequence;
}

bool MfPreselectionFlowAction::canPickup() const {
  return MfPreselectionLogicResult::canPickup(pickup_count_,
                                              params_.max_pickup_count);
}

MfPreselectionFlowAction::Phase
MfPreselectionFlowAction::detectionMissNextPhase() const {
  switch (detect_mode_) {
  case DetectMode::Entry2:
    return Phase::EntryHighRaise;
  case DetectMode::Stair1:
    return Phase::EntryMoveRightToStair3;
  case DetectMode::Stair3:
    return Phase::EntryReturnFromStair3;
  case DetectMode::RowFront:
    if (direct_exit_mode_) {
      return (current_grid_ == 11) ? Phase::Row4ForcedTurn : Phase::TransitionTurn;
    }
    if (current_grid_ == 2) {
      return Phase::TransitionTurn;
    }
    if (current_grid_ == 11) {
      return Phase::Row4ForcedTurn;
    }
    return Phase::RowScanTurnLeft;
  case DetectMode::Scan:
    return active_detection_phase_ == Phase::RowScanDetectLeft
               ? Phase::RowScanTurnBack
               : Phase::RowAlignExit;
  case DetectMode::TransitionObserve:
    return Phase::TransitionStair;
  case DetectMode::Row4Fake:
    return Phase::Row4FakeTurnBack;
  }
  return Phase::AfterEntry;
}

void MfPreselectionFlowAction::rememberPickupSource(
    MfPreselectionPickupSource source) {
  // 只有非 None 来源会覆盖 pickup_source_。路线中途发现并夹取的 KFS 不改变
  // 入口侧来源，否则假 KFS 避障会丢掉入口探测时建立的左右侧判断。
  if (source != MfPreselectionPickupSource::None) {
    pickup_source_ = source;
  }
  if (config().blackboard) {
    config().blackboard->set("mf_preselect_pickup_source",
                             std::string(sourceName(pickup_source_)));
  }
}

void MfPreselectionFlowAction::writeBlackboardState(const std::string &state) {
  // 黑板状态不是公开 ROS 接口，只给同一棵树和日志诊断使用；但每次阶段切换
  // 都维护它，便于实车观察当前卡在“检测/等待/移动/台阶”的哪类语义上。
  if (!config().blackboard) {
    return;
  }
  config().blackboard->set("mf_preselect_state", state);
  config().blackboard->set("mf_preselect_pickup_count", pickup_count_);
  config().blackboard->set("mf_preselect_current_grid", current_grid_);
}

const char *MfPreselectionFlowAction::detectModeText(DetectMode mode) {
  switch (mode) {
  case DetectMode::Entry2:
    return "入口2号阶梯检测";
  case DetectMode::Stair1:
    return "入口1号阶梯检测";
  case DetectMode::Stair3:
    return "入口3号阶梯检测";
  case DetectMode::RowFront:
    return "当前行前方检测";
  case DetectMode::Scan:
    return "周身扫描检测";
  case DetectMode::Row4Fake:
    return "第四行假KFS检测";
  case DetectMode::TransitionObserve:
    return "高低阶梯切换前方观察";
  }
  return "未知检测";
}

const char *MfPreselectionFlowAction::stairModeText(StairMode mode) {
  return mode == StairMode::Climb ? "上阶梯" : "下阶梯";
}

const char *MfPreselectionFlowAction::wheelEventText(WheelEvent event) {
  switch (event) {
  case WheelEvent::FrontFirst:
    return "前轮第一激光高度突变";
  case WheelEvent::FrontSecond:
    return "前轮第二激光高度突变";
  case WheelEvent::Rear:
    return "后轮激光高度突变";
  }
  return "未知激光事件";
}

const char *MfPreselectionFlowAction::phaseText(Phase phase) {
  switch (phase) {
  case Phase::EntryDetectStair2:
    return "入口2号检测";
  case Phase::EntryHighRaise:
    return "入口机械臂高位抬升";
  case Phase::EntryMoveLeft:
    return "入口左移探测";
  case Phase::EntryDetectStair1:
    return "入口1号检测";
  case Phase::EntryMoveRightToStair3:
    return "入口右移到3号";
  case Phase::EntryDetectStair3:
    return "入口3号检测";
  case Phase::EntryPrepareClimb:
    return "入口上阶准备";
  case Phase::EntryClimb:
    return "入口上阶";
  case Phase::EntryReturnFromStair1:
    return "入口从1号回中";
  case Phase::EntryReturnFromStair3:
    return "入口从3号回中";
  case Phase::EntryReturnToCenterAfterInterruptedPickup:
    return "入口中途夹取后回中";
  case Phase::EntryResumeInterruptedProbeMove:
    return "入口恢复被中断横移";
  case Phase::AfterEntry:
    return "进入梅林内部决策";
  case Phase::RowFrontDetect:
    return "行前方检测";
  case Phase::RowScanTurnLeft:
    return "周身扫描左转";
  case Phase::RowScanDetectLeft:
    return "周身扫描左侧检测";
  case Phase::RowScanTurnBack:
    return "周身扫描背向转向";
  case Phase::RowScanDetectBack:
    return "周身扫描背向检测";
  case Phase::RowAlignExit:
    return "重新对齐出口方向";
  case Phase::FakeAvoidTurn:
    return "假KFS避障转向";
  case Phase::FakeAvoidArmAdjust:
    return "假KFS避障机械臂调整";
  case Phase::FakeAvoidStair:
    return "假KFS避障台阶动作";
  case Phase::FakeAvoidAlignExit:
    return "假KFS避障后对齐出口";
  case Phase::FakeAvoidForwardStep:
    return "假KFS旁列前向推进";
  case Phase::TransitionStair:
    return "格间台阶动作";
  case Phase::TransitionTurn:
    return "格间转向";
  case Phase::TransitionArmAdjust:
    return "格间机械臂调整";
  case Phase::DetectionArmAdjust:
    return "检测前机械臂调整";
  case Phase::TransitionObserve:
    return "格间前方观察";
  case Phase::Row4ForcedTurn:
    return "第四行强制转向";
  case Phase::Row4DetectFake:
    return "第四行假KFS检测";
  case Phase::Row4FakeTurnBack:
    return "第四行假KFS转回";
  case Phase::FinalExitYawAlign:
    return "最终离场朝向对齐";
  case Phase::Row4DirectDescendPrep:
    return "第四行直接下阶准备";
  case Phase::Row4DirectDescend:
    return "第四行直接下阶";
  case Phase::DirectExitDrive:
    return "直行离场兜底";
  case Phase::FinalStop:
    return "最终停车等待";
  case Phase::EntryRetryBackoff:
    return "入口夹取失败后退重试";
  case Phase::RetryPostGrabCenterAlign:
    return "夹取失败后归中重试";
  case Phase::KfsVisualAlign:
    return "KFS视觉横移对齐";
  case Phase::KfsSecondArmLower:
    return "KFS第二节机械臂下降";
  case Phase::KfsOdomApproach:
    return "KFS odom前向趋近";
  case Phase::MechanismCommand:
    return "机构命令等待";
  case Phase::GrabVerify:
    return "夹取视觉验证";
  case Phase::MoveRelative:
    return "相对移动";
  case Phase::TurnYaw:
    return "转向";
  case Phase::ZeroHold:
    return "零速等待";
  case Phase::StairPrimitive:
    return "台阶原语";
  case Phase::PostGrabCenterAlign:
    return "夹取后格中心归位";
  case Phase::CenterAlign:
    return "格中心归位";
  case Phase::Done:
    return "完成";
  default:
    return "未知阶段";
  }
}

void MfPreselectionFlowAction::beginPreparedDetection(DetectMode mode,
                                                      double timeout_s,
                                                      Phase detection_phase) {
  pending_detection_mode_ = mode;
  pending_detection_phase_ = detection_phase;

  bool high_side = true;
  int target_grid = 0;
  int height_delta = 0;
  const bool resolved =
      resolveDetectionHighSide(mode, detection_phase, high_side, target_grid,
                               height_delta);
  pending_detection_high_side_ = high_side;

  if (mode == DetectMode::Scan) {
    active_detection_phase_ = detection_phase;
  }

  if (node_) {
    if (resolved) {
      RCLCPP_INFO(node_->get_logger(),
                  "梅林预选赛准备检测前机械臂姿态：%s current_grid=%d target_grid=%d height_delta=%d high_side=%s",
                  detectModeText(mode), current_grid_, target_grid, height_delta,
                  high_side ? "是" : "否");
    } else {
      RCLCPP_WARN(node_->get_logger(),
                  "梅林预选赛无法解析检测目标高度：%s current_grid=%d，保守按高侧检测",
                  detectModeText(mode), current_grid_);
    }
  }

  if (arm_high_side_ == high_side) {
    beginDetection(mode, timeout_s);
    return;
  }

  phase_ = Phase::DetectionArmAdjust;
}

bool MfPreselectionFlowAction::resolveDetectionHighSide(
    DetectMode mode, Phase detection_phase, bool &high_side, int &target_grid,
    int &height_delta) const {
  target_grid = current_grid_;
  height_delta = 0;

  switch (mode) {
  case DetectMode::RowFront:
    if (current_grid_ == 2 || current_grid_ == 5 || current_grid_ == 8 ||
        current_grid_ == 11) {
      target_grid = current_grid_ + 3;
    } else {
      high_side = true;
      return false;
    }
    break;
  case DetectMode::Scan:
    if (detection_phase == Phase::RowScanDetectLeft) {
      target_grid = current_grid_ - 1;
    } else if (detection_phase == Phase::RowScanDetectBack) {
      target_grid = current_grid_ + 1;
    } else {
      high_side = true;
      return false;
    }
    break;
  case DetectMode::TransitionObserve:
    target_grid = transition_target_grid_;
    break;
  case DetectMode::Entry2:
  case DetectMode::Stair1:
  case DetectMode::Stair3:
  case DetectMode::Row4Fake:
    high_side = true;
    return true;
  }

  std::shared_ptr<MerlinMapManager> map;
  if (!config().blackboard || !config().blackboard->get("merlin_map", map) ||
      !map) {
    high_side = true;
    return false;
  }

  const int from_depth = map->getDepth(current_grid_);
  const int target_depth = map->getDepth(target_grid);
  if (from_depth < 0 || target_depth < 0) {
    high_side = true;
    return false;
  }

  height_delta = target_depth - from_depth;
  high_side = height_delta >= 0;
  return true;
}

void MfPreselectionFlowAction::beginDetection(DetectMode mode,
                                              double timeout_s) {
  // beginDetection() 只初始化检测窗口；真正的视觉判断在 tickDetection()
  // 每个 BT tick 里完成。这样 R1 等待、目标稳定帧和超时都不会阻塞树线程。
  detect_mode_ = mode;
  if (mode == DetectMode::Scan &&
      (phase_ == Phase::RowScanDetectLeft ||
       phase_ == Phase::RowScanDetectBack)) {
    // Scan 模式跨两个方向复用同一个 detect_mode_，active_detection_phase_
    // 记录当前是在“左侧检测”还是“背向检测”，决定超时后的下一步。
    active_detection_phase_ = phase_;
  } else {
    active_detection_phase_ = Phase::Done;
  }
  resetDetectionCounters();
  detection_active_ = true;
  if (node_) {
    phase_start_ = node_->now();
  }
  switch (mode) {
  case DetectMode::Entry2:
    phase_ = Phase::EntryDetectStair2;
    writeBlackboardState("entry_detect_stair2");
    break;
  case DetectMode::Stair1:
    phase_ = Phase::EntryDetectStair1;
    writeBlackboardState("entry_detect_stair1");
    break;
  case DetectMode::Stair3:
    phase_ = Phase::EntryDetectStair3;
    writeBlackboardState("entry_detect_stair3");
    break;
  case DetectMode::RowFront:
    phase_ = Phase::RowFrontDetect;
    writeBlackboardState("row_front_detect");
    break;
  case DetectMode::Scan:
    if (active_detection_phase_ != Phase::RowScanDetectLeft &&
        active_detection_phase_ != Phase::RowScanDetectBack) {
      active_detection_phase_ = Phase::RowScanDetectLeft;
    }
    phase_ = active_detection_phase_;
    writeBlackboardState("row_scan_detect");
    break;
  case DetectMode::Row4Fake:
    phase_ = Phase::Row4DetectFake;
    writeBlackboardState("row4_fake_detect");
    break;
  case DetectMode::TransitionObserve:
    phase_ = Phase::TransitionObserve;
    writeBlackboardState("transition_observe");
    break;
  }

  if (mode == DetectMode::TransitionObserve) {
    active_detection_high_side_ = transition_high_side_;
  } else {
    bool high_side = true;
    int target_grid = 0;
    int height_delta = 0;
    (void)resolveDetectionHighSide(mode, phase_, high_side, target_grid,
                                   height_delta);
    active_detection_high_side_ = high_side;
  }

  if (node_) {
    RCLCPP_INFO(node_->get_logger(),
                "梅林预选赛进入检测阶段：%s，grid=%d，高侧=%s，直出模式=%s，已夹取=%d/%d，超时=%.2fs",
                detectModeText(mode), current_grid_,
                active_detection_high_side_ ? "是" : "否",
                direct_exit_mode_ ? "是" : "否", pickup_count_,
                params_.max_pickup_count, timeout_s);
  }
}

BT::NodeStatus MfPreselectionFlowAction::tickDetection() {
  // 某些 phase 是从 onRunning() 直接切过来的，可能尚未调用 beginDetection()。
  // 这里做一次自恢复初始化，保证检测阶段不会因状态切换顺序漏掉定时器和计数器。
  if (!detection_active_) {
    if (phase_ == Phase::RowScanDetectLeft) {
      beginPreparedDetection(DetectMode::Scan, params_.scan_detect_timeout_s,
                             Phase::RowScanDetectLeft);
    } else if (phase_ == Phase::RowScanDetectBack) {
      beginPreparedDetection(DetectMode::Scan, params_.scan_detect_timeout_s,
                             Phase::RowScanDetectBack);
    } else if (phase_ == Phase::RowFrontDetect) {
      beginPreparedDetection(DetectMode::RowFront,
                             params_.scan_detect_timeout_s,
                             Phase::RowFrontDetect);
    } else if (phase_ == Phase::TransitionObserve) {
      beginDetection(DetectMode::TransitionObserve, params_.scan_detect_timeout_s);
    } else if (phase_ == Phase::Row4DetectFake) {
      beginDetection(DetectMode::Row4Fake, params_.scan_detect_timeout_s);
    }
    return BT::NodeStatus::RUNNING;
  }
  publishStop();
  const double elapsed =
      node_ ? (node_->now() - phase_start_).seconds() : 0.0;
  const double timeout =
      (detect_mode_ == DetectMode::Entry2) ? params_.entry_detect_timeout_s
                                           : params_.scan_detect_timeout_s;

  if (detect_mode_ == DetectMode::Row4Fake) {
    // 第四行收尾只关心“转向后正前方是否是假 KFS”。这里不再处理 R2/R1，
    // 因为路线已经进入离场决策：有假 KFS 则回头下阶梯，否则继续 grid12。
    const auto fake = findFakeTarget();
    if (fake.has_value()) {
      RCLCPP_WARN(node_->get_logger(),
                  "梅林预选赛第四行转向后检测到假KFS：label=%s distance=%.3fm，执行180度转向后下阶梯离场",
                  fake->label.c_str(), fake->distance_m);
      row4_fake_detected_ = true;
      detection_active_ = false;
      phase_ = Phase::Row4FakeTurnBack;
      return BT::NodeStatus::RUNNING;
    }
    if (elapsed >= timeout) {
      RCLCPP_INFO(node_->get_logger(),
                  "梅林预选赛第四行转向后未检测到假KFS，继续常规 grid11 -> grid12 离场");
      row4_fake_detected_ = false;
      detection_active_ = false;
      phase_ = Phase::TransitionTurn;
      return BT::NodeStatus::RUNNING;
    }
    return BT::NodeStatus::RUNNING;
  }

  if (detect_mode_ != DetectMode::Scan) {
    if ((detect_mode_ == DetectMode::RowFront ||
         detect_mode_ == DetectMode::TransitionObserve) &&
        guardPathObstacles()) {
      // 前方检测/台阶观察阶段看到 R1 时，守卫函数会持续停车等待；
      // 周身扫描阶段故意不等 R1，因为扫描看到的 R1 不一定挡住当前行进路径。
      return BT::NodeStatus::RUNNING;
    }
  }

  const bool entry_detection =
      detect_mode_ == DetectMode::Entry2 || detect_mode_ == DetectMode::Stair1 ||
      detect_mode_ == DetectMode::Stair3;
  const auto r2 = findR2LockObservation(
      entry_detection ? R2DepthProfile::Entry : R2DepthProfile::General);
  if (r2.has_value()) {
    if (r2->target.sequence != last_detection_sequence_) {
      // 只有新帧才累计 stable 计数，避免同一帧在高 tick 率下被重复计算。
      last_detection_sequence_ = r2->target.sequence;
      ++detect_seen_count_;
      detect_lost_count_ = 0;
    }
    if (detect_seen_count_ >= params_.detect_seen_stable_frames) {
      RCLCPP_INFO(node_->get_logger(),
                  "梅林预选赛检测到R2 KFS并锁定单帧：阶段=%s label=%s seq=%ld distance=%s depth_source=%s score=%.3f offset=%dpx target_line_offset=%dpx stable=%d/%d bbox=[%.1f %.1f %.1f %.1f]",
                  detectModeText(detect_mode_), r2->target.label.c_str(),
                  static_cast<long>(r2->target.sequence),
                  metersText(r2->has_depth, r2->target.distance_m).c_str(),
                  MfPreselectionLogicResult::kfsDepthSourceText(
                      r2->depth_source),
                  r2->target.score, r2->offset_px,
                  params_.kfs_align_target_line_offset_px, detect_seen_count_,
                  params_.detect_seen_stable_frames, r2->target.x1,
                  r2->target.y1, r2->target.x2, r2->target.y2);
      switch (detect_mode_) {
      case DetectMode::Entry2:
        // 2 号入口正前方夹取后直接准备上首阶，来源记录为 Stair2。
        // 2 号入口使用普通高侧夹取 GRAB_KFS_UP(0x03)，但仍保留
        // Stair2 来源用于后续入口来源和假 KFS 避障语义。
        beginKfsVisualPickup(true, MfPreselectionPickupSource::Stair2,
                             *r2, Phase::EntryPrepareClimb,
                             detectionMissNextPhase(), false, false,
                             R2DepthProfile::Entry);
        break;
      case DetectMode::Stair1:
        // 1/3 号入口夹取完成后先横移回 2 号入口，再统一入场。
        beginKfsVisualPickup(true, MfPreselectionPickupSource::Stair1,
                             *r2, Phase::EntryReturnFromStair1,
                             detectionMissNextPhase(), false, true,
                             R2DepthProfile::Entry);
        break;
      case DetectMode::Stair3:
        beginKfsVisualPickup(true, MfPreselectionPickupSource::Stair3,
                             *r2, Phase::EntryReturnFromStair3,
                             detectionMissNextPhase(), false, true,
                             R2DepthProfile::Entry);
        break;
      case DetectMode::RowFront:
      case DetectMode::Scan:
        // 入场后再次发现 R2 KFS，夹取后进入直出模式：不再做第 2/3 行周身
        // 搜索，但仍保留前方守卫和假 KFS 处理。
        beginKfsVisualPickup(active_detection_high_side_,
                             MfPreselectionPickupSource::None, *r2,
                             Phase::AfterEntry,
                             detectionMissNextPhase(), true, false);
        break;
      case DetectMode::TransitionObserve:
        // 台阶切换前方观察到 R2 KFS 时，按本次台阶高低侧决定夹取命令，
        // 夹取完成后继续原计划台阶动作。
        beginKfsVisualPickup(transition_high_side_,
                             MfPreselectionPickupSource::None, *r2,
                             Phase::TransitionStair, detectionMissNextPhase(),
                             false, false);
        break;
      case DetectMode::Row4Fake:
        break;
      }
      detection_active_ = false;
      return BT::NodeStatus::RUNNING;
    }
    return BT::NodeStatus::RUNNING;
  }

  if (detect_mode_ == DetectMode::RowFront && current_grid_ != 11) {
    // 第 1/2/3 行前方假 KFS 触发避障；第 4 行假 KFS 有专门的 Row4Fake
    // 收尾逻辑，不能混用这里的上阶梯绕行动作。
    const auto fake = findFakeTarget();
    if (fake.has_value()) {
      RCLCPP_WARN(node_->get_logger(),
                  "梅林预选赛当前行前方检测到假KFS：grid=%d label=%s distance=%.3fm，进入假KFS避障",
                  current_grid_, fake->label.c_str(), fake->distance_m);
      detection_active_ = false;
      resetDetectionCounters();
      active_detection_phase_ = Phase::Done;
      phase_ = Phase::FakeAvoidTurn;
      return BT::NodeStatus::RUNNING;
    }
  }

  const auto latest_sequence = latestVisionSequence();
  if ((!latest_sequence.has_value() ||
       *latest_sequence == last_detection_sequence_) &&
      elapsed < timeout) {
    // 没有新帧且检测窗口还没超时：保持 RUNNING，既不累计 lost，也不切阶段。
    return BT::NodeStatus::RUNNING;
  }
  if (latest_sequence.has_value()) {
    last_detection_sequence_ = *latest_sequence;
  }
  ++detect_lost_count_;
  detect_seen_count_ = 0;
  if (elapsed < timeout) {
    // “未发现”必须等完整检测窗口结束。否则 RealSense 已稳定出帧后，
    // 连续几帧无 R2 会让旁列前方观察在 0.5s 左右提前结束，实车上表现为
    // 刚到 grid4/grid7 就直接转向下阶梯。
    return BT::NodeStatus::RUNNING;
  }
  if (detect_lost_count_ < params_.detect_lost_stable_frames) {
    // 检测窗口到时后仍保留稳定丢失帧确认，避免刚好到时的单帧漏检
    // 直接切走。
    return BT::NodeStatus::RUNNING;
  }

  switch (detect_mode_) {
  case DetectMode::Entry2:
    // 中间入口未发现目标后才高抬升并探侧边，减少不必要的横移。
  {
    const std::string reject_summary = r2LockRejectSummary();
    RCLCPP_INFO(node_->get_logger(),
                "梅林预选赛入口2号未发现R2 KFS，准备发送高抬升命令并横移探测1/3号阶梯%s",
                reject_summary.c_str());
    phase_ = Phase::EntryHighRaise;
    break;
  }
  case DetectMode::Stair1:
  {
    const std::string reject_summary = r2LockRejectSummary();
    RCLCPP_INFO(node_->get_logger(),
                "梅林预选赛入口1号未发现R2 KFS，准备横移到3号阶梯探测%s",
                reject_summary.c_str());
    phase_ = Phase::EntryMoveRightToStair3;
    break;
  }
  case DetectMode::Stair3:
  {
    const std::string reject_summary = r2LockRejectSummary();
    RCLCPP_INFO(node_->get_logger(),
                "梅林预选赛入口3号未发现R2 KFS，准备返回2号入口上阶梯%s",
                reject_summary.c_str());
    phase_ = Phase::EntryReturnFromStair3;
    break;
  }
  case DetectMode::RowFront:
  {
    const std::string reject_summary = r2LockRejectSummary();
    RCLCPP_INFO(node_->get_logger(),
                "梅林预选赛当前行前方未发现可处理目标：grid=%d，直出模式=%s%s",
                current_grid_, direct_exit_mode_ ? "是" : "否",
                reject_summary.c_str());
    // 第一行不做周身扫描，直接向下一格推进；第 2/3 行未夹取时才进入扫描。
    if (direct_exit_mode_) {
      phase_ = (current_grid_ == 11) ? Phase::Row4ForcedTurn
                                    : Phase::TransitionTurn;
    } else if (current_grid_ == 2) {
      phase_ = Phase::TransitionTurn;
    } else if (current_grid_ == 11) {
      phase_ = Phase::Row4ForcedTurn;
    } else {
      phase_ = Phase::RowScanTurnLeft;
    }
    break;
  }
  case DetectMode::Scan:
  {
    const std::string reject_summary = r2LockRejectSummary();
    if (active_detection_phase_ == Phase::RowScanDetectLeft) {
      RCLCPP_INFO(node_->get_logger(),
                  "梅林预选赛周身扫描左侧未发现R2 KFS，继续背向扫描%s",
                  reject_summary.c_str());
      phase_ = Phase::RowScanTurnBack;
    } else {
      RCLCPP_INFO(node_->get_logger(),
                  "梅林预选赛周身扫描未发现R2 KFS，重新朝向出口方向并继续推进%s",
                  reject_summary.c_str());
      phase_ = Phase::RowAlignExit;
    }
    break;
  }
  case DetectMode::TransitionObserve:
  {
    const std::string reject_summary = r2LockRejectSummary();
    RCLCPP_INFO(node_->get_logger(),
                "梅林预选赛高低阶梯切换前方观察未发现R2 KFS，继续执行格间台阶动作%s",
                reject_summary.c_str());
    phase_ = Phase::TransitionStair;
    break;
  }
  case DetectMode::Row4Fake:
    break;
  }
  detection_active_ = false;
  return BT::NodeStatus::RUNNING;
}

void MfPreselectionFlowAction::resetDetectionCounters() {
  // seen/lost 计数只在一个检测窗口内有效；跨阶段复用会让目标稳定性判断串味。
  detect_seen_count_ = 0;
  detect_lost_count_ = 0;
  last_detection_sequence_ = 0;
  kfs_align_target_lock_state_.reset();
  kfs_align_last_observation_.reset();
  kfs_align_target_lock_sequence_ = 0;
  r2_lock_logged_reasons_this_detection_.clear();
  clearR2LockReject();
}

void MfPreselectionFlowAction::beginMechanismCommand(
    uint8_t command_id, std::string label, int done_feedback_id,
    Phase next_phase, std::string failure_reason) {
  // 单条机构命令分两层完成条件：
  // - service response accepted=true 表示 transport 已接收/ACK；
  // - done_feedback_id >= 0 时，还必须等同 seq 的完成反馈。
  // 推杆命令通常只等 ACK；机械臂升降这类姿态动作需要 done 反馈。
  command_generation_.fetch_add(1, std::memory_order_relaxed);
  command_id_ = command_id;
  command_label_ = std::move(label);
  command_done_feedback_id_ = done_feedback_id;
  command_next_phase_ = next_phase;
  command_failure_reason_ = std::move(failure_reason);
  command_response_seen_ = false;
  command_accepted_ = false;
  command_seq_ = -1;
  command_done_seen_ = done_feedback_id < 0;
  command_sent_ = false;
  command_waiting_service_logged_ = false;
  command_ack_logged_ = false;
  command_waiting_done_logged_ = false;
  if (node_) {
    phase_start_ = node_->now();
    if (done_feedback_id >= 0) {
      RCLCPP_INFO(node_->get_logger(),
                  "梅林预选赛准备下发机构命令：%s(0x%02X)，等待完成反馈=0x%02X",
                  command_label_.c_str(),
                  static_cast<unsigned int>(command_id_),
                  static_cast<unsigned int>(done_feedback_id));
    } else {
      RCLCPP_INFO(node_->get_logger(),
                  "梅林预选赛准备下发机构命令：%s(0x%02X)，仅等待ACK",
                  command_label_.c_str(),
                  static_cast<unsigned int>(command_id_));
    }
  }
  phase_ = Phase::MechanismCommand;
}

BT::NodeStatus MfPreselectionFlowAction::tickMechanismCommand() {
  publishStop();
  if (!node_ || !send_client_) {
    return fail(command_failure_reason_);
  }
  if (command_response_seen_.load(std::memory_order_relaxed)) {
    if (!command_accepted_.load(std::memory_order_relaxed)) {
      if (command_failure_reason_ == "grab_kfs_failed" &&
          scheduleGrabRetryAfterVisibleFailure(command_failure_reason_)) {
        pending_grab_commit_ = false;
        pending_grab_source_ = MfPreselectionPickupSource::None;
        pending_grab_entry_high_protocol_ = false;
        pending_grab_target_.reset();
        grab_success_direct_exit_ = false;
        post_grab_center_next_phase_ = Phase::Done;
        return BT::NodeStatus::RUNNING;
      }
      return fail(command_failure_reason_);
    }
    if (!command_ack_logged_) {
      RCLCPP_INFO(node_->get_logger(),
                  "梅林预选赛机构命令ACK成功：%s(0x%02X)，seq=%d",
                  command_label_.c_str(),
                  static_cast<unsigned int>(command_id_),
                  command_seq_.load(std::memory_order_relaxed));
      command_ack_logged_ = true;
    }
    if (command_done_seen_.load(std::memory_order_relaxed)) {
      if (command_next_phase_ == Phase::GrabVerify && pending_grab_commit_) {
        beginGrabVerify();
        return BT::NodeStatus::RUNNING;
      }
      if (pending_grab_commit_) {
        // 夹取计数只在命令 ACK/完成后提交，避免 service rejected 或超时时
        // 黑板提前显示已经夹取。
        commitPendingGrab();
      }
      if (command_next_phase_ == Phase::StairPrimitive) {
        // 台阶原语通过“发送机构命令 -> MechanismCommand phase -> 回到
        // StairPrimitive”来复用通用 service 等待逻辑。这里把对应子阶段推进到
        // 命令完成后的 hold/drive 阶段。
        if (stair_phase_ == StairPhase::ClimbSendFrontExtend) {
          stair_phase_ = StairPhase::ClimbHoldAfterFrontExtend;
        } else if (stair_phase_ == StairPhase::ClimbSendRearRetract) {
          stair_phase_ = StairPhase::ClimbHoldAfterRearRetract;
        } else if (stair_phase_ == StairPhase::DescendSendRearExtend) {
          stair_phase_ = StairPhase::DescendHoldAfterRearExtend;
        } else if (stair_phase_ == StairPhase::DescendSendFrontRetract) {
          stair_phase_ = StairPhase::DescendHoldAfterFrontRetract;
        }
      }
      RCLCPP_INFO(node_->get_logger(),
                  "梅林预选赛机构命令完成：%s，进入下一阶段",
                  command_label_.c_str());
      if (command_next_phase_ == Phase::ZeroHold) {
        // beginGrab() 会把夹取命令的 next_phase 设为 ZeroHold，并预先填好
        // zero_hold_*，让机械爪闭合后留出一个 settle 时间再继续路线。
        if (node_) {
          phase_start_ = node_->now();
          RCLCPP_INFO(node_->get_logger(),
                      "梅林预选赛开始零速等待：%s，duration=%.2fs",
                      zero_hold_label_.c_str(), zero_hold_duration_s_);
        }
        phase_ = Phase::ZeroHold;
        return BT::NodeStatus::RUNNING;
      }
      phase_ = command_next_phase_;
      return BT::NodeStatus::RUNNING;
    }
    if (!command_waiting_done_logged_) {
      RCLCPP_INFO(node_->get_logger(),
                  "梅林预选赛机构命令ACK后等待完成反馈：%s，feedback=0x%02X，seq=%d",
                  command_label_.c_str(),
                  static_cast<unsigned int>(command_done_feedback_id_),
                  command_seq_.load(std::memory_order_relaxed));
      command_waiting_done_logged_ = true;
    }
  }
  if ((node_->now() - phase_start_).seconds() > params_.command_timeout_s) {
    if (command_failure_reason_ == "grab_kfs_failed" &&
        scheduleGrabRetryAfterVisibleFailure(command_failure_reason_)) {
      pending_grab_commit_ = false;
      pending_grab_source_ = MfPreselectionPickupSource::None;
      pending_grab_entry_high_protocol_ = false;
      pending_grab_target_.reset();
      grab_success_direct_exit_ = false;
      post_grab_center_next_phase_ = Phase::Done;
      return BT::NodeStatus::RUNNING;
    }
    return fail(command_failure_reason_);
  }
  if (!command_sent_) {
    if (!send_client_->service_is_ready()) {
      if (!command_waiting_service_logged_) {
        RCLCPP_WARN(node_->get_logger(),
                    "梅林预选赛等待机构命令服务可用：%s，service=%s",
                    command_label_.c_str(), params_.send_command_service.c_str());
        command_waiting_service_logged_ = true;
      }
      return BT::NodeStatus::RUNNING;
    }
    RCLCPP_INFO(node_->get_logger(),
                "梅林预选赛已发送机构命令请求：%s(0x%02X)",
                command_label_.c_str(),
                static_cast<unsigned int>(command_id_));
    auto request = std::make_shared<SendCommandSrv::Request>();
    request->command_id = command_id_;
    request->payload.clear();
    const uint64_t token =
        command_generation_.load(std::memory_order_relaxed);
    // async_send_request 的回调可能晚于 halt/restart；token 检查保证旧请求不会
    // 污染新一轮 command_* 状态。
    send_client_->async_send_request(
        request, [this, token](rclcpp::Client<SendCommandSrv>::SharedFuture future) {
          if (token != command_generation_.load(std::memory_order_relaxed)) {
            return;
          }
          try {
            const auto response = future.get();
            command_accepted_.store(response && response->accepted,
                                    std::memory_order_relaxed);
            if (response) {
              command_seq_.store(static_cast<int>(response->seq),
                                 std::memory_order_relaxed);
            }
          } catch (...) {
            command_accepted_.store(false, std::memory_order_relaxed);
          }
          command_response_seen_.store(true, std::memory_order_relaxed);
        });
    command_sent_ = true;
  }
  return BT::NodeStatus::RUNNING;
}

void MfPreselectionFlowAction::beginCommandPair(
    uint8_t first_id, std::string first_label, uint8_t second_id,
    std::string second_label, Phase next_phase, std::string failure_reason) {
  // 台阶中有两处需要近似同时切换前后推杆。这里并发发送两条 service 请求，
  // 只等待各自 accepted，不等待 done feedback；机械动作间隔由后续 ZeroHold 控制。
  command_generation_.fetch_add(1, std::memory_order_relaxed);
  command_pair_[0].command_id = first_id;
  command_pair_[0].label = std::move(first_label);
  command_pair_[0].sent = false;
  command_pair_[0].response_seen = false;
  command_pair_[0].accepted = false;
  command_pair_[0].seq = -1;
  command_pair_ack_logged_[0] = false;
  command_pair_[1].command_id = second_id;
  command_pair_[1].label = std::move(second_label);
  command_pair_[1].sent = false;
  command_pair_[1].response_seen = false;
  command_pair_[1].accepted = false;
  command_pair_[1].seq = -1;
  command_pair_ack_logged_[1] = false;
  command_pair_active_ = true;
  command_pair_waiting_service_logged_ = false;
  command_next_phase_ = next_phase;
  command_pair_failure_reason_ = std::move(failure_reason);
  if (node_) {
    phase_start_ = node_->now();
    RCLCPP_INFO(node_->get_logger(),
                "梅林预选赛准备并发下发机构命令：%s(0x%02X) + %s(0x%02X)",
                command_pair_[0].label.c_str(),
                static_cast<unsigned int>(command_pair_[0].command_id),
                command_pair_[1].label.c_str(),
                static_cast<unsigned int>(command_pair_[1].command_id));
  }
  phase_ = Phase::MechanismCommand;
}

BT::NodeStatus MfPreselectionFlowAction::tickCommandPair() {
  publishStop();
  if (!node_ || !send_client_ || !command_pair_active_) {
    return fail(command_pair_failure_reason_);
  }
  for (const auto &slot : command_pair_) {
    if (slot.response_seen.load(std::memory_order_relaxed) &&
        !slot.accepted.load(std::memory_order_relaxed)) {
      return fail(command_pair_failure_reason_);
    }
  }
  const bool first_done =
      command_pair_[0].response_seen.load(std::memory_order_relaxed) &&
      command_pair_[0].accepted.load(std::memory_order_relaxed);
  const bool second_done =
      command_pair_[1].response_seen.load(std::memory_order_relaxed) &&
      command_pair_[1].accepted.load(std::memory_order_relaxed);
  for (std::size_t index = 0; index < 2; ++index) {
    if (!command_pair_ack_logged_[index] &&
        command_pair_[index].response_seen.load(std::memory_order_relaxed) &&
        command_pair_[index].accepted.load(std::memory_order_relaxed)) {
      RCLCPP_INFO(node_->get_logger(),
                  "梅林预选赛并发机构命令ACK成功：%s(0x%02X)，seq=%d",
                  command_pair_[index].label.c_str(),
                  static_cast<unsigned int>(command_pair_[index].command_id),
                  command_pair_[index].seq.load(std::memory_order_relaxed));
      command_pair_ack_logged_[index] = true;
    }
  }
  if (first_done && second_done) {
    command_pair_active_ = false;
    if (command_next_phase_ == Phase::StairPrimitive) {
      // 与单命令一样，双命令完成后需要把台阶子状态推进到对应的等待阶段。
      if (stair_phase_ == StairPhase::ClimbSendFrontRetractAndRearExtend) {
        stair_phase_ = StairPhase::ClimbHoldAfterFrontRetractAndRearExtend;
      } else if (stair_phase_ ==
                 StairPhase::DescendSendRearRetractAndFrontExtend) {
        stair_phase_ = StairPhase::DescendHoldAfterRearRetractAndFrontExtend;
      }
    }
    RCLCPP_INFO(
        node_->get_logger(),
        "梅林预选赛并发机构命令均ACK成功：%s seq=%d + %s seq=%d，进入下一阶段",
        command_pair_[0].label.c_str(),
        command_pair_[0].seq.load(std::memory_order_relaxed),
        command_pair_[1].label.c_str(),
        command_pair_[1].seq.load(std::memory_order_relaxed));
    phase_ = command_next_phase_;
    return BT::NodeStatus::RUNNING;
  }
  if ((node_->now() - phase_start_).seconds() > params_.command_timeout_s) {
    return fail(command_pair_failure_reason_);
  }
  if (!send_client_->service_is_ready()) {
    if (!command_pair_waiting_service_logged_) {
      RCLCPP_WARN(node_->get_logger(),
                  "梅林预选赛等待机构命令服务可用：并发命令，service=%s",
                  params_.send_command_service.c_str());
      command_pair_waiting_service_logged_ = true;
    }
    return BT::NodeStatus::RUNNING;
  }

  const uint64_t token =
      command_generation_.load(std::memory_order_relaxed);
  for (std::size_t index = 0; index < 2; ++index) {
    auto &slot = command_pair_[index];
    if (slot.sent) {
      continue;
    }
    auto request = std::make_shared<SendCommandSrv::Request>();
    request->command_id = slot.command_id;
    request->payload.clear();
    slot.sent = true;
    RCLCPP_INFO(node_->get_logger(),
                "梅林预选赛已发送并发机构命令请求：%s(0x%02X)",
                slot.label.c_str(), static_cast<unsigned int>(slot.command_id));
    // 并发命令的两个回调各写自己的 slot；token 仍用于屏蔽旧流程回调。
    send_client_->async_send_request(
        request, [this, token, index](rclcpp::Client<SendCommandSrv>::SharedFuture future) {
          if (token != command_generation_.load(std::memory_order_relaxed)) {
            return;
          }
          bool accepted = false;
          try {
            const auto response = future.get();
            accepted = response && response->accepted;
            if (response) {
              command_pair_[index].seq.store(static_cast<int>(response->seq),
                                             std::memory_order_relaxed);
            }
          } catch (...) {
            accepted = false;
          }
          command_pair_[index].accepted.store(accepted, std::memory_order_relaxed);
          command_pair_[index].response_seen.store(true, std::memory_order_relaxed);
        });
  }
  return BT::NodeStatus::RUNNING;
}

void MfPreselectionFlowAction::beginMoveRelative(double vx, double vy,
                                                 double distance_m,
                                                 Phase next_phase,
                                                 std::string label) {
  // 相对移动只按 odom 起点和欧氏位移闭环，不规划全局路径。入口横移、
  // 直出兜底和短距离离场都使用这个原语。
  move_vx_ = vx;
  move_vy_ = vy;
  move_distance_m_ = std::max(0.0, distance_m);
  move_next_phase_ = next_phase;
  move_label_ = std::move(label);
  move_start_captured_ = false;
  move_waiting_odom_logged_ = false;
  if (node_) {
    phase_start_ = node_->now();
    RCLCPP_INFO(node_->get_logger(),
                "梅林预选赛开始相对移动：%s，vx=%.3f vy=%.3f distance=%.3fm",
                move_label_.c_str(), move_vx_, move_vy_, move_distance_m_);
  }
  phase_ = Phase::MoveRelative;
}

void MfPreselectionFlowAction::continueAfterMoveRelative(Phase next_phase) {
  phase_ = next_phase;
  if (phase_ == Phase::EntryDetectStair1) {
    // 入口探测横移完成后立即进入对应检测窗口，不等下一次 onRunning()
    // 再做模式映射。
    beginDetection(DetectMode::Stair1, params_.scan_detect_timeout_s);
  } else if (phase_ == Phase::EntryDetectStair3) {
    beginDetection(DetectMode::Stair3, params_.scan_detect_timeout_s);
  } else if (phase_ == Phase::FinalStop) {
    beginZeroHold(0.5, Phase::Done, "final_stop_hold");
  }
}

bool MfPreselectionFlowAction::isEntryInterruptibleMove() const {
  return move_label_ == "entry_probe_left" ||
         move_label_ == "entry_probe_stair3" ||
         move_label_ == "entry_return_from_stair1" ||
         move_label_ == "entry_return_from_stair3" ||
         move_label_ == "entry_return_to_center_after_interrupted_pickup";
}

std::optional<double> MfPreselectionFlowAction::entryMoveTargetOffset() const {
  const double mirror_sign = static_cast<double>(params_.field_mirror_sign);
  if (move_label_ == "entry_probe_left") {
    return mirror_sign * params_.entry_probe_left_distance_m;
  }
  if (move_label_ == "entry_probe_stair3") {
    return mirror_sign * (params_.entry_probe_left_distance_m -
                          params_.entry_probe_right_sweep_distance_m);
  }
  if (move_label_ == "entry_return_from_stair1" ||
      move_label_ == "entry_return_from_stair3" ||
      move_label_ == "entry_return_to_center_after_interrupted_pickup") {
    return 0.0;
  }
  return std::nullopt;
}

bool MfPreselectionFlowAction::captureEntryLateralReferenceIfNeeded() {
  if (entry_lateral_reference_captured_) {
    return true;
  }
  if (!odomReady()) {
    return false;
  }
  if (move_label_ == "entry_probe_left") {
    entry_lateral_reference_x_ = move_start_captured_ ? move_start_x_ : odom_x_;
    entry_lateral_reference_y_ = move_start_captured_ ? move_start_y_ : odom_y_;
    entry_lateral_reference_yaw_ =
        move_start_captured_ ? move_start_yaw_ : odom_yaw_;
    entry_lateral_reference_captured_ = true;
    RCLCPP_INFO(node_->get_logger(),
                "梅林预选赛入口横向参考已记录：x=%.3f y=%.3f yaw=%.3f",
                entry_lateral_reference_x_, entry_lateral_reference_y_,
                entry_lateral_reference_yaw_);
    return true;
  }
  return false;
}

double MfPreselectionFlowAction::currentEntryLateralOffset() const {
  const double dx = odom_x_ - entry_lateral_reference_x_;
  const double dy = odom_y_ - entry_lateral_reference_y_;
  const double left_x = -std::sin(entry_lateral_reference_yaw_);
  const double left_y = std::cos(entry_lateral_reference_yaw_);
  return dx * left_x + dy * left_y;
}

bool MfPreselectionFlowAction::maybeInterruptEntryMoveForKfs() {
  if (!isEntryInterruptibleMove() || !canPickup()) {
    return false;
  }
  if (!captureEntryLateralReferenceIfNeeded() ||
      !entry_lateral_reference_captured_) {
    return false;
  }
  const auto target_offset = entryMoveTargetOffset();
  if (!target_offset.has_value()) {
    return false;
  }
  const auto r2 = findR2LockObservation(R2DepthProfile::Entry);
  if (!r2.has_value() ||
      r2->target.sequence == entry_move_last_interrupt_sequence_) {
    return false;
  }

  const double lateral_speed = move_vy_;
  const int dynamic_extra_px =
      MfPreselectionLogicResult::entryInterruptDynamicExtraPx(
          lateral_speed, r2->target.distance_m, params_);
  const int effective_limit_px =
      MfPreselectionLogicResult::entryInterruptEffectiveOffsetLimitPx(
          lateral_speed, r2->target.distance_m, params_);
  if (!MfPreselectionLogicResult::entryInterruptOffsetAcceptable(
          r2->offset_px, lateral_speed, r2->target.distance_m, params_)) {
    entry_move_last_interrupt_sequence_ = r2->target.sequence;
    if (node_) {
      RCLCPP_INFO(
          node_->get_logger(),
          "梅林预选赛入口横移中看到R2 KFS但暂不停车：move=%s label=%s seq=%ld image_offset=%dpx base_limit=%dpx dynamic_extra=%dpx effective_limit=%dpx vy=%.3fm/s depth=%s depth_source=%s，继续横移等待目标进入可夹取窗口",
          move_label_.c_str(), r2->target.label.c_str(),
          static_cast<long>(r2->target.sequence), r2->offset_px,
          params_.entry_interrupt_max_offset_px, dynamic_extra_px,
          effective_limit_px, lateral_speed,
          metersText(r2->has_depth, r2->target.distance_m).c_str(),
          MfPreselectionLogicResult::kfsDepthSourceText(r2->depth_source));
    }
    return false;
  }

  publishStop();
  entry_move_last_interrupt_sequence_ = r2->target.sequence;
  entry_move_interrupted_active_ = true;
  interrupted_entry_move_next_phase_ = move_next_phase_;
  interrupted_entry_move_label_ = move_label_;
  interrupted_entry_move_target_offset_m_ = *target_offset;

  const double lateral_offset = currentEntryLateralOffset();
  MfPreselectionPickupSource source =
      MfPreselectionLogicResult::entryPickupSourceForLateralOffset(
          lateral_offset, params_.move_tolerance_m,
          params_.field_mirror_sign);
  if (pickup_source_ != MfPreselectionPickupSource::None) {
    source = MfPreselectionPickupSource::None;
  }

  RCLCPP_INFO(node_->get_logger(),
              "梅林预选赛入口横移中单帧锁定R2 KFS：move=%s label=%s seq=%ld lateral_offset=%.3fm image_offset=%dpx base_limit=%dpx dynamic_extra=%dpx effective_limit=%dpx vy=%.3fm/s depth=%s depth_source=%s source=%s，立即停车进入视觉横移对齐夹取",
              move_label_.c_str(), r2->target.label.c_str(),
              static_cast<long>(r2->target.sequence), lateral_offset,
              r2->offset_px, params_.entry_interrupt_max_offset_px,
              dynamic_extra_px, effective_limit_px, lateral_speed,
              metersText(r2->has_depth, r2->target.distance_m).c_str(),
              MfPreselectionLogicResult::kfsDepthSourceText(r2->depth_source),
              sourceName(source));
  beginKfsVisualPickup(true, source, *r2,
                       Phase::EntryReturnToCenterAfterInterruptedPickup,
                       Phase::EntryResumeInterruptedProbeMove, false, true,
                       R2DepthProfile::Entry);
  beginEntryMcuStopSettle(lateral_speed);
  return true;
}

void MfPreselectionFlowAction::beginEntryMcuStopSettle(
    double lateral_speed_mps) {
  kfs_entry_mcu_stop_settle_active_ = false;
  kfs_entry_mcu_stop_settle_done_logged_ = false;
  kfs_entry_mcu_stop_settle_duration_s_ =
      MfPreselectionLogicResult::entryMcuStopSettleDuration(lateral_speed_mps,
                                                            params_);
  kfs_entry_mcu_stop_settle_speed_mps_ = lateral_speed_mps;
  if (!node_ || kfs_entry_mcu_stop_settle_duration_s_ <= 0.0) {
    return;
  }

  kfs_entry_mcu_stop_settle_active_ = true;
  kfs_entry_mcu_stop_settle_until_ =
      node_->now() + seconds(kfs_entry_mcu_stop_settle_duration_s_);
  publishStop();
  RCLCPP_INFO(
      node_->get_logger(),
      "梅林预选赛入口横移中断后进入MCU减速等待：vy=%.3fm/s acc=%.3fm/s^2 wait=%.3fs margin=%.3fs max_wait=%.3fs",
      lateral_speed_mps, params_.entry_mcu_vy_acc_mps2,
      kfs_entry_mcu_stop_settle_duration_s_, params_.entry_mcu_stop_margin_s,
      params_.entry_mcu_stop_max_wait_s);
}

bool MfPreselectionFlowAction::tickEntryMcuStopSettle() {
  if (!kfs_entry_mcu_stop_settle_active_) {
    return false;
  }
  publishStop();
  if (!node_) {
    kfs_entry_mcu_stop_settle_active_ = false;
    return false;
  }
  if (node_->now() < kfs_entry_mcu_stop_settle_until_) {
    return true;
  }

  kfs_entry_mcu_stop_settle_active_ = false;
  if (!kfs_entry_mcu_stop_settle_done_logged_) {
    kfs_entry_mcu_stop_settle_done_logged_ = true;
    // 等待 MCU 收完入口扫线速度后再开始计算视觉对齐总超时。
    kfs_align_total_start_ = node_->now();
    RCLCPP_INFO(
        node_->get_logger(),
        "梅林预选赛入口MCU减速等待完成：vy=%.3fm/s wait=%.3fs，开始KFS视觉横移对齐",
        kfs_entry_mcu_stop_settle_speed_mps_,
        kfs_entry_mcu_stop_settle_duration_s_);
  }
  return false;
}

BT::NodeStatus
MfPreselectionFlowAction::beginEntryReturnToCenterAfterInterruptedPickup() {
  if (!entry_move_interrupted_active_) {
    phase_ = Phase::EntryPrepareClimb;
    return BT::NodeStatus::RUNNING;
  }
  if (!node_ || !odomReady() || !entry_lateral_reference_captured_) {
    return fail("entry_return_to_center_reference_missing");
  }

  double vy = 0.0;
  double distance_m = 0.0;
  const double lateral_offset = currentEntryLateralOffset();
  if (!MfPreselectionLogicResult::entryReturnToCenterCommand(
          lateral_offset, params_.move_tolerance_m,
          params_.lateral_probe_speed_mps, vy, distance_m)) {
    return fail("entry_return_to_center_invalid_offset");
  }

  entry_move_interrupted_active_ = false;
  if (distance_m <= 0.0) {
    RCLCPP_INFO(node_->get_logger(),
                "梅林预选赛入口中途夹取后已在2号入口横向容差内：offset=%.3fm，直接准备上阶梯",
                lateral_offset);
    phase_ = Phase::EntryPrepareClimb;
    return BT::NodeStatus::RUNNING;
  }

  const double raw_distance_m = distance_m;
  double compensation_m = 0.0;
  distance_m =
      MfPreselectionLogicResult::entryReturnToCenterCompensatedDistance(
          raw_distance_m, vy, params_, compensation_m);
  if (distance_m <= params_.move_tolerance_m) {
    RCLCPP_INFO(
        node_->get_logger(),
        "梅林预选赛入口中途夹取成功后回2号入口距离经MCU减速+延迟补偿后已无需横移：offset=%.3fm vy=%.3f raw_distance=%.3fm compensation=%.3fm distance=%.3fm tolerance=%.3fm",
        lateral_offset, vy, raw_distance_m, compensation_m, distance_m,
        params_.move_tolerance_m);
    publishStop();
    phase_ = Phase::EntryPrepareClimb;
    return BT::NodeStatus::RUNNING;
  }

  RCLCPP_INFO(node_->get_logger(),
              "梅林预选赛入口中途夹取成功后回2号入口：offset=%.3fm vy=%.3f raw_distance=%.3fm compensation=%.3fm distance=%.3fm",
              lateral_offset, vy, raw_distance_m, compensation_m, distance_m);
  beginMoveRelative(0.0, vy, distance_m, Phase::EntryPrepareClimb,
                    "entry_return_to_center_after_interrupted_pickup");
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus MfPreselectionFlowAction::resumeInterruptedEntryMove() {
  if (!entry_move_interrupted_active_) {
    phase_ = interrupted_entry_move_next_phase_;
    continueAfterMoveRelative(phase_);
    return BT::NodeStatus::RUNNING;
  }
  if (!node_ || !odomReady() || !entry_lateral_reference_captured_) {
    return fail("entry_resume_probe_reference_missing");
  }

  const double lateral_offset = currentEntryLateralOffset();
  const double remaining =
      interrupted_entry_move_target_offset_m_ - lateral_offset;
  entry_move_interrupted_active_ = false;
  if (std::abs(remaining) <= params_.move_tolerance_m) {
    RCLCPP_INFO(node_->get_logger(),
                "梅林预选赛入口横移夹取未成功但已到原目标横向位置：move=%s offset=%.3fm target=%.3fm",
                interrupted_entry_move_label_.c_str(), lateral_offset,
                interrupted_entry_move_target_offset_m_);
    continueAfterMoveRelative(interrupted_entry_move_next_phase_);
    return BT::NodeStatus::RUNNING;
  }

  const double vy =
      remaining > 0.0 ? std::abs(params_.lateral_probe_speed_mps)
                      : -std::abs(params_.lateral_probe_speed_mps);
  RCLCPP_INFO(node_->get_logger(),
              "梅林预选赛入口横移夹取未成功，补完原横移：move=%s offset=%.3fm target=%.3fm vy=%.3f distance=%.3fm",
              interrupted_entry_move_label_.c_str(), lateral_offset,
              interrupted_entry_move_target_offset_m_, vy,
              std::abs(remaining));
  beginMoveRelative(0.0, vy, std::abs(remaining),
                    interrupted_entry_move_next_phase_,
                    interrupted_entry_move_label_);
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus MfPreselectionFlowAction::tickMoveRelative() {
  if (!node_) {
    return fail("move_runtime_missing");
  }
  if (guardPathObstacles()) {
    // R1 阻挡期间保持停车，并重置相对移动超时窗口；等待人工机器人让开
    // 不应被算入本段移动超时。
    phase_start_ = node_->now();
    return BT::NodeStatus::RUNNING;
  }
  if (!odomReady()) {
    publishStop();
    if (!move_waiting_odom_logged_) {
      RCLCPP_WARN(node_->get_logger(),
                  "梅林预选赛相对移动等待odom新鲜：%s，odom_topic=%s",
                  move_label_.c_str(), params_.odom_topic.c_str());
      move_waiting_odom_logged_ = true;
    }
    return BT::NodeStatus::RUNNING;
  }
  if (!move_start_captured_) {
    // 首次拿到新鲜 odom 后再捕获起点，避免用启动前的默认 0 位姿计算距离。
    move_start_x_ = odom_x_;
    move_start_y_ = odom_y_;
    move_start_yaw_ = odom_yaw_;
    move_start_captured_ = true;
    if (node_) {
      phase_start_ = node_->now();
      RCLCPP_INFO(node_->get_logger(),
                  "梅林预选赛相对移动已捕获起点：%s，x=%.3f y=%.3f yaw=%.3f",
                  move_label_.c_str(), move_start_x_, move_start_y_,
                  move_start_yaw_);
    }
  }
  if (move_start_captured_ && isEntryInterruptibleMove()) {
    (void)captureEntryLateralReferenceIfNeeded();
    if (maybeInterruptEntryMoveForKfs()) {
      return BT::NodeStatus::RUNNING;
    }
  }
  const double dx = odom_x_ - move_start_x_;
  const double dy = odom_y_ - move_start_y_;
  const double traveled = std::hypot(dx, dy);
  if (traveled + params_.move_tolerance_m >= move_distance_m_) {
    // 完成后清掉 R1 等待状态，避免下一段移动继承上一段的 lost 计数。
    publishStop();
    clearPathR1Wait();
    RCLCPP_INFO(node_->get_logger(),
                "梅林预选赛相对移动完成：%s，已行驶=%.3fm 目标=%.3fm",
                move_label_.c_str(), traveled, move_distance_m_);
    continueAfterMoveRelative(move_next_phase_);
    return BT::NodeStatus::RUNNING;
  }
  if ((node_->now() - phase_start_).seconds() > params_.move_timeout_s) {
    return fail("move_timeout_" + move_label_);
  }
  // 发布线速度的同时叠加启动 yaw 的 heading hold，抵消横移/直行时的车身漂角。
  publishTwist(move_vx_, move_vy_, headingAngularZ(move_start_yaw_));
  return BT::NodeStatus::RUNNING;
}

void MfPreselectionFlowAction::beginDirectExitDrive() {
  // direct_exit 是路线兜底：当已经夹取或假 KFS 避障完成后，不再做完整行扫描，
  // 只按参数距离朝出口方向直行，同时保留前方 R1/R2/假 KFS 守卫。
  if (node_) {
    RCLCPP_INFO(node_->get_logger(),
                "梅林预选赛进入专用直行离场兜底：distance=%.3fm speed=%.3fm/s",
                params_.direct_exit_drive_distance_m,
                params_.direct_exit_drive_speed_mps);
  }
  beginMoveRelative(params_.direct_exit_drive_speed_mps, 0.0,
                    params_.direct_exit_drive_distance_m, Phase::FinalStop,
                    "direct_exit");
  phase_ = Phase::DirectExitDrive;
  direct_exit_move_active_ = true;
}

BT::NodeStatus MfPreselectionFlowAction::tickDirectExitDrive() {
  if (!direct_exit_move_active_) {
    beginDirectExitDrive();
  }
  if (guardPathObstacles()) {
    // 与普通相对移动一样，R1 等待不计入直出移动超时。
    phase_start_ = node_->now();
    return BT::NodeStatus::RUNNING;
  }
  if (canPickup()) {
    const auto r2 = findR2LockObservation();
    if (r2.has_value()) {
    // 直出途中仍然允许补夹 R2 KFS，但不记录新的入口来源。
      RCLCPP_INFO(node_->get_logger(),
                  "梅林预选赛直行离场途中单帧锁定R2 KFS：label=%s offset=%dpx depth=%s depth_source=%s，先夹取后继续离场",
                  r2->target.label.c_str(), r2->offset_px,
                  metersText(r2->has_depth, r2->target.distance_m).c_str(),
                  MfPreselectionLogicResult::kfsDepthSourceText(
                      r2->depth_source));
      beginKfsVisualPickup(true, MfPreselectionPickupSource::None, *r2,
                           Phase::DirectExitDrive, Phase::DirectExitDrive,
                           false, false);
      return BT::NodeStatus::RUNNING;
    }
  }
  if (current_grid_ != 11 && findFakeTarget().has_value()) {
    // grid11 的假 KFS 已由第四行专属逻辑处理；其它位置看到假 KFS 走通用避障。
    RCLCPP_WARN(node_->get_logger(),
                "梅林预选赛直行离场途中检测到假KFS，切入假KFS避障");
    direct_exit_move_active_ = false;
    phase_ = Phase::FakeAvoidTurn;
    return BT::NodeStatus::RUNNING;
  }
  return tickMoveRelative();
}

bool MfPreselectionFlowAction::guardPathObstacles() {
  // R1 是人工机器人需要处理的阻挡目标。只要路径前方稳定看到 R1，本车就
  // 零速等待；默认没有总超时，除非现场显式配置 path_r1_lost_wait_timeout_s。
  const auto r1 = findR1BlockingTarget();
  if (r1.has_value()) {
    publishStop();
    writeBlackboardState("waiting_r1_blocker");
    if (!path_r1_waiting_) {
      path_r1_waiting_ = true;
      path_r1_lost_count_ = 0;
      path_r1_last_sequence_ = r1->sequence;
      if (node_) {
        path_r1_wait_start_ = node_->now();
      }
      RCLCPP_WARN(node_->get_logger(),
                  "梅林预选赛路径前方遇到R1阻挡，原地等待人工机器人处理：label=%s distance=%.3fm",
                  r1->label.c_str(), r1->distance_m);
    } else if (r1->sequence != path_r1_last_sequence_) {
      // 新帧仍看到 R1，刷新序号并清空“已丢失帧”计数。
      path_r1_last_sequence_ = r1->sequence;
      path_r1_lost_count_ = 0;
    }
    return true;
  }
  if (!path_r1_waiting_) {
    return false;
  }
  if (params_.path_r1_lost_wait_timeout_s > 0.0 && node_ &&
      (node_->now() - path_r1_wait_start_).seconds() >
          params_.path_r1_lost_wait_timeout_s) {
    RCLCPP_WARN(node_->get_logger(),
                "梅林预选赛R1等待达到配置超时，按参数放行：timeout=%.2fs",
                params_.path_r1_lost_wait_timeout_s);
    clearPathR1Wait();
    return false;
  }
  const auto latest_sequence = latestVisionSequence();
  if (!latest_sequence.has_value() ||
      *latest_sequence == path_r1_last_sequence_) {
    // 没有新视觉帧时继续等，不把同一帧重复当成 R1 丢失。
    publishStop();
    writeBlackboardState("waiting_r1_blocker");
    return true;
  }
  path_r1_last_sequence_ = *latest_sequence;
  ++path_r1_lost_count_;
  if (path_r1_lost_count_ >= params_.detect_lost_stable_frames) {
    // R1 连续丢失若干新帧后才解除等待，过滤单帧漏检。
    RCLCPP_INFO(node_->get_logger(),
                "梅林预选赛R1阻挡已连续丢失%d帧，解除等待继续流程",
                path_r1_lost_count_);
    clearPathR1Wait();
    return false;
  }
  publishStop();
  writeBlackboardState("waiting_r1_blocker");
  return true;
}

void MfPreselectionFlowAction::clearPathR1Wait() {
  // 路径守卫状态跨 tick 保持，但跨动作段必须清理。
  path_r1_waiting_ = false;
  path_r1_lost_count_ = 0;
  path_r1_last_sequence_ = 0;
}

bool MfPreselectionFlowAction::lastGrabOriginPathBlocking() const {
  return last_grab_origin_phase_ == Phase::DirectExitDrive ||
         last_grab_origin_detect_mode_ == DetectMode::RowFront ||
         last_grab_origin_detect_mode_ == DetectMode::TransitionObserve;
}

void MfPreselectionFlowAction::rememberCurrentKfsPickupForRetry() {
  last_grab_retry_context_valid_ = true;
  last_grab_high_side_ = kfs_pickup_high_side_;
  last_grab_source_ = kfs_pickup_source_;
  last_grab_success_phase_ = kfs_pickup_success_phase_;
  last_grab_failure_phase_ = kfs_pickup_failure_phase_;
  last_grab_direct_exit_on_success_ = kfs_pickup_direct_exit_on_success_;
  last_grab_entry_high_protocol_ = kfs_pickup_entry_high_protocol_;
  last_grab_depth_profile_ = kfs_pickup_depth_profile_;
  last_grab_approach_distance_m_ = kfs_odom_approach_distance_m_;
  last_grab_target_ = kfs_locked_target_.has_value()
                          ? kfs_locked_target_
                          : kfs_pickup_initial_target_;
  last_grab_origin_phase_ = kfs_pickup_origin_phase_;
  last_grab_origin_detect_mode_ = kfs_pickup_origin_detect_mode_;
}

bool MfPreselectionFlowAction::scheduleGrabRetryAfterVisibleFailure(
    const std::string &reason) {
  if (!last_grab_retry_context_valid_ || !last_grab_target_.has_value()) {
    return false;
  }
  if (reason != "grab_verify_target_still_visible" &&
      !mandatoryEntryStair2RetryActive()) {
    return false;
  }

  const auto retry_action = MfPreselectionLogicResult::grabRetryAction(
      true, last_grab_source_, last_grab_entry_high_protocol_,
      lastGrabOriginPathBlocking());
  if (retry_action == MfPreselectionLogicResult::GrabRetryAction::None) {
    return false;
  }

  retry_grab_context_valid_ = true;
  retry_grab_backoff_started_ = false;
  retry_grab_high_side_ = last_grab_high_side_;
  retry_grab_source_ = last_grab_source_;
  retry_grab_success_phase_ = last_grab_success_phase_;
  retry_grab_failure_phase_ = last_grab_failure_phase_;
  retry_grab_direct_exit_on_success_ = last_grab_direct_exit_on_success_;
  retry_grab_entry_high_protocol_ = last_grab_entry_high_protocol_;
  retry_grab_depth_profile_ = last_grab_depth_profile_;
  retry_grab_approach_distance_m_ = last_grab_approach_distance_m_;
  retry_grab_target_ = last_grab_target_;
  retry_grab_origin_phase_ = last_grab_origin_phase_;
  retry_grab_origin_detect_mode_ = last_grab_origin_detect_mode_;

  if (node_) {
    RCLCPP_WARN(
        node_->get_logger(),
        "梅林预选赛夹取未确认成功，调度重新夹取：reason=%s action=%s source=%s origin=%s detect=%s approach=%.3fm target=%s seq=%ld",
        translateMfFailureReason(reason).c_str(),
        retry_action == MfPreselectionLogicResult::GrabRetryAction::EntryBackoff
            ? "entry_backoff"
            : "grid_center_retry",
        sourceName(retry_grab_source_), phaseText(retry_grab_origin_phase_),
        detectModeText(retry_grab_origin_detect_mode_),
        retry_grab_approach_distance_m_, retry_grab_target_->label.c_str(),
        static_cast<long>(retry_grab_target_->sequence));
  }

  phase_ =
      retry_action == MfPreselectionLogicResult::GrabRetryAction::EntryBackoff
          ? Phase::EntryRetryBackoff
          : Phase::RetryPostGrabCenterAlign;
  return true;
}

bool MfPreselectionFlowAction::mandatoryEntryStair2RetryActive() const {
  return MfPreselectionLogicResult::mandatoryEntryStair2Retry(
      last_grab_source_, last_grab_entry_high_protocol_);
}

BT::NodeStatus MfPreselectionFlowAction::tickEntryRetryBackoff() {
  if (!node_ || !retry_grab_context_valid_) {
    return fail("grab_retry_context_missing");
  }

  if (!retry_grab_backoff_started_) {
    const double backoff_distance = -retry_grab_approach_distance_m_;
    retry_grab_backoff_started_ = true;
    if (std::abs(backoff_distance) > params_.kfs_approach_odom_tolerance_m) {
      RCLCPP_WARN(
          node_->get_logger(),
          "梅林预选赛入口夹取失败后退到可重新识别位置：distance=%.3fm target=%s",
          backoff_distance,
          retry_grab_target_.has_value() ? retry_grab_target_->label.c_str()
                                         : "");
      beginKfsOdomAxisMotion(KfsOdomAxis::X, backoff_distance,
                             params_.kfs_approach_speed_mps,
                             params_.kfs_approach_min_speed_mps,
                             params_.kfs_approach_odom_tolerance_m,
                             params_.kfs_approach_timeout_s,
                             "entry_grab_retry_backoff");
      return BT::NodeStatus::RUNNING;
    }
    RCLCPP_INFO(
        node_->get_logger(),
        "梅林预选赛入口夹取失败但前向趋近距离在容差内，直接重新识别夹取：distance=%.3fm",
        backoff_distance);
  } else if (kfs_odom_motion_started_) {
    std::string failure_reason;
    const auto motion_result = tickKfsOdomAxisMotion(failure_reason);
    if (motion_result == KfsOdomMotionResult::Failed) {
      return fail(failure_reason.empty() ? "kfs_odom_approach_runtime_failed"
                                         : failure_reason);
    }
    if (motion_result == KfsOdomMotionResult::Running) {
      return BT::NodeStatus::RUNNING;
    }
    publishStop();
    clearKfsOdomAxisMotion();
  }

  const auto observation = findR2LockObservation(retry_grab_depth_profile_);
  if (!observation.has_value()) {
    publishStop();
    writeBlackboardState("entry_grab_retry_wait_target");
    return BT::NodeStatus::RUNNING;
  }

  const auto high_side = retry_grab_high_side_;
  const auto source = retry_grab_source_;
  const auto success_phase = retry_grab_success_phase_;
  const auto failure_phase = retry_grab_failure_phase_;
  const auto direct_exit_on_success = retry_grab_direct_exit_on_success_;
  const auto entry_high_protocol = retry_grab_entry_high_protocol_;
  const auto depth_profile = retry_grab_depth_profile_;
  RCLCPP_INFO(node_->get_logger(),
              "梅林预选赛入口夹取失败后重新识别到R2 KFS，继续夹取：label=%s seq=%ld",
              observation->target.label.c_str(),
              static_cast<long>(observation->target.sequence));
  clearGrabRetryContext();
  beginKfsVisualPickup(high_side, source, *observation, success_phase,
                       failure_phase, direct_exit_on_success,
                       entry_high_protocol, depth_profile);
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus MfPreselectionFlowAction::beginRetryPostGrabCenterAlign() {
  if (!node_ || !retry_grab_context_valid_) {
    return fail("grab_retry_context_missing");
  }

  Phase next_phase = retry_grab_origin_phase_;
  if (next_phase == Phase::Done || next_phase == Phase::KfsVisualAlign ||
      next_phase == Phase::KfsOdomApproach || next_phase == Phase::GrabVerify) {
    next_phase = retry_grab_origin_detect_mode_ == DetectMode::TransitionObserve
                     ? Phase::TransitionObserve
                     : Phase::RowFrontDetect;
  }
  if (next_phase == Phase::DirectExitDrive) {
    direct_exit_move_active_ = false;
  }
  const double target_yaw = postGrabCenterYaw();
  RCLCPP_WARN(node_->get_logger(),
              "梅林预选赛梅林内夹取失败且原目标仍可见，先归当前格中心后重新观察：grid=%d next=%s yaw=%.3f",
              current_grid_, phaseText(next_phase), target_yaw);
  if (!beginGridCenterAlign(current_grid_, target_yaw, next_phase,
                            "retry_after_failed_grab_center")) {
    return fail("grab_retry_center_start_failed");
  }
  clearGrabRetryContext();
  writeBlackboardState("grab_retry_center_align");
  return BT::NodeStatus::RUNNING;
}

void MfPreselectionFlowAction::clearGrabRetryContext() {
  retry_grab_context_valid_ = false;
  retry_grab_backoff_started_ = false;
  retry_grab_high_side_ = true;
  retry_grab_source_ = MfPreselectionPickupSource::None;
  retry_grab_success_phase_ = Phase::Done;
  retry_grab_failure_phase_ = Phase::Done;
  retry_grab_direct_exit_on_success_ = false;
  retry_grab_entry_high_protocol_ = false;
  retry_grab_depth_profile_ = R2DepthProfile::General;
  retry_grab_approach_distance_m_ = 0.0;
  retry_grab_target_.reset();
  retry_grab_origin_phase_ = Phase::Done;
  retry_grab_origin_detect_mode_ = DetectMode::Entry2;
  last_grab_retry_context_valid_ = false;
  last_grab_high_side_ = true;
  last_grab_source_ = MfPreselectionPickupSource::None;
  last_grab_success_phase_ = Phase::Done;
  last_grab_failure_phase_ = Phase::Done;
  last_grab_direct_exit_on_success_ = false;
  last_grab_entry_high_protocol_ = false;
  last_grab_depth_profile_ = R2DepthProfile::General;
  last_grab_approach_distance_m_ = 0.0;
  last_grab_target_.reset();
  last_grab_origin_phase_ = Phase::Done;
  last_grab_origin_detect_mode_ = DetectMode::Entry2;
}

void MfPreselectionFlowAction::beginTurnYaw(double target_yaw_rad,
                                            Phase next_phase,
                                            std::string label) {
  // 转向原语只闭环 yaw，不做平移；所有目标 yaw 都在调用方根据路线语义算好。
  turn_target_yaw_ = normalizeAngle(target_yaw_rad);
  turn_next_phase_ = next_phase;
  turn_label_ = std::move(label);
  turn_stable_ticks_ = 0;
  turn_waiting_odom_logged_ = false;
  if (node_) {
    phase_start_ = node_->now();
    RCLCPP_INFO(node_->get_logger(),
                "梅林预选赛开始转向：%s，target_yaw=%.3f rad",
                turn_label_.c_str(), turn_target_yaw_);
  }
  phase_ = Phase::TurnYaw;
}

BT::NodeStatus MfPreselectionFlowAction::tickTurnYaw() {
  if (!node_) {
    return fail("turn_runtime_missing");
  }
  if ((node_->now() - phase_start_).seconds() > params_.turn_timeout_s) {
    return fail("turn_timeout_" + turn_label_);
  }
  if (!odomReady()) {
    publishStop();
    if (!turn_waiting_odom_logged_) {
      RCLCPP_WARN(node_->get_logger(),
                  "梅林预选赛转向等待odom新鲜：%s，odom_topic=%s",
                  turn_label_.c_str(), params_.odom_topic.c_str());
      turn_waiting_odom_logged_ = true;
    }
    return BT::NodeStatus::RUNNING;
  }
  const double error = normalizeAngle(turn_target_yaw_ - odom_yaw_);
  if (std::abs(error) <= params_.turn_tolerance_deg * kDeg2Rad) {
    // 连续稳定若干 tick 才算完成，避免速度刚降到零附近时因 odom 抖动提前放行。
    ++turn_stable_ticks_;
    publishStop();
    if (turn_stable_ticks_ >= params_.turn_stable_ticks) {
      RCLCPP_INFO(node_->get_logger(),
                  "梅林预选赛转向完成：%s，当前yaw=%.3f target=%.3f error=%.3f rad",
                  turn_label_.c_str(), odom_yaw_, turn_target_yaw_, error);
      phase_ = turn_next_phase_;
    }
    return BT::NodeStatus::RUNNING;
  }
  turn_stable_ticks_ = 0;
  // yaw P 控制限幅后直接发布角速度；转向阶段不叠加线速度。
  const double wz =
      std::clamp(params_.turn_kp * error, -params_.turn_max_speed_radps,
                 params_.turn_max_speed_radps);
  publishTwist(0.0, 0.0, wz);
  return BT::NodeStatus::RUNNING;
}

void MfPreselectionFlowAction::beginZeroHold(double duration_s,
                                             Phase next_phase,
                                             std::string label) {
  // ZeroHold 是机械动作后的显式稳定时间，也是最终停车保持的短暂确认窗口。
  zero_hold_duration_s_ = std::max(0.0, duration_s);
  zero_hold_next_phase_ = next_phase;
  zero_hold_label_ = std::move(label);
  if (node_) {
    phase_start_ = node_->now();
    RCLCPP_INFO(node_->get_logger(),
                "梅林预选赛开始零速等待：%s，duration=%.2fs",
                zero_hold_label_.c_str(), zero_hold_duration_s_);
  }
  phase_ = Phase::ZeroHold;
}

BT::NodeStatus MfPreselectionFlowAction::tickZeroHold() {
  publishStop();
  if (!node_ ||
      (node_->now() - phase_start_).seconds() >= zero_hold_duration_s_) {
    if (node_) {
      RCLCPP_INFO(node_->get_logger(),
                  "梅林预选赛零速等待完成：%s，进入下一阶段",
                  zero_hold_label_.c_str());
    }
    phase_ = zero_hold_next_phase_;
  }
  return BT::NodeStatus::RUNNING;
}

bool MfPreselectionFlowAction::beginEntryCenterAdvance(Phase next_phase,
                                                       std::string label) {
  center_policy_ = StairCenterPolicy::EntryGrid2Reference;
  center_next_phase_ = next_phase;
  center_label_ = std::move(label);
  center_target_grid_ = 2;
  center_target_yaw_ = normalizeAngle(entry_heading_yaw_);
  center_stable_ticks_ = 0;
  center_target_ready_ = false;
  center_waiting_odom_logged_ = false;
  setCenterError("");
  writeBlackboardState("entry_center_align");
  if (config().blackboard) {
    config().blackboard->set("mf_center_target_grid", center_target_grid_);
  }
  if (node_) {
    phase_start_ = node_->now();
    RCLCPP_INFO(node_->get_logger(),
                "梅林预选赛开始入口格中心前进：%s，reference_grid=2 yaw=%.3f offset=%.3fm",
                center_label_.c_str(), center_target_yaw_,
                center_params_.entry_forward_offset_m);
  }
  phase_ = Phase::CenterAlign;
  return true;
}

bool MfPreselectionFlowAction::beginGridCenterAlign(int target_grid,
                                                    double target_yaw_rad,
                                                    Phase next_phase,
                                                    std::string label) {
  if (!validGrid(target_grid)) {
    return false;
  }
  center_policy_ = StairCenterPolicy::TransitionTargetGrid;
  center_next_phase_ = next_phase;
  center_label_ = std::move(label);
  center_target_grid_ = target_grid;
  center_target_yaw_ = normalizeAngle(target_yaw_rad);
  center_stable_ticks_ = 0;
  center_target_ready_ = false;
  center_waiting_odom_logged_ = false;
  setCenterError("");
  if (!computeGridCenterFromReference(target_grid, center_target_x_,
                                      center_target_y_)) {
    setCenterError("missing_center_reference");
    return false;
  }
  center_target_ready_ = true;
  writeCenterTargetBlackboard();
  writeBlackboardState("grid_center_align");
  if (node_) {
    phase_start_ = node_->now();
    RCLCPP_INFO(node_->get_logger(),
                "梅林预选赛开始格中心归位：%s，grid%d x=%.3f y=%.3f yaw=%.3f",
                center_label_.c_str(), center_target_grid_, center_target_x_,
                center_target_y_, center_target_yaw_);
  }
  phase_ = Phase::CenterAlign;
  return true;
}

bool MfPreselectionFlowAction::beginFinalExitCenterAlign(Phase next_phase,
                                                        std::string label) {
  int reference_grid = 0;
  double reference_x = 0.0;
  double reference_y = 0.0;
  double reference_yaw = 0.0;
  if (!readCenterReference(reference_grid, reference_x, reference_y,
                           reference_yaw) ||
      !computeGridCenterFromReference(current_grid_, reference_x,
                                      reference_y)) {
    setCenterError("missing_final_exit_center_reference");
    return false;
  }
  if (!MfPreselectionLogicResult::finalExitCenterTarget(
          reference_x, reference_y, entry_heading_yaw_,
          params_.final_exit_center_offset_m, center_target_x_,
          center_target_y_)) {
    setCenterError("invalid_final_exit_center_target");
    return false;
  }

  center_policy_ = StairCenterPolicy::FinalExitVirtual;
  center_next_phase_ = next_phase;
  center_label_ = std::move(label);
  center_target_grid_ = kFinalExitVirtualGrid;
  center_target_yaw_ = normalizeAngle(entry_heading_yaw_);
  center_stable_ticks_ = 0;
  center_target_ready_ = true;
  center_waiting_odom_logged_ = false;
  setCenterError("");
  writeCenterTargetBlackboard();
  writeBlackboardState("final_exit_center_align");
  if (node_) {
    phase_start_ = node_->now();
    RCLCPP_INFO(node_->get_logger(),
                "梅林预选赛开始最终离场虚拟归位：%s，从grid%d中心外推 %.3fm 到 x=%.3f y=%.3f yaw=%.3f",
                center_label_.c_str(), current_grid_,
                params_.final_exit_center_offset_m, center_target_x_,
                center_target_y_, center_target_yaw_);
  }
  phase_ = Phase::CenterAlign;
  return true;
}

BT::NodeStatus MfPreselectionFlowAction::beginPostGrabCenterAlign() {
  const Phase next_phase = post_grab_center_next_phase_;
  const double target_yaw = postGrabCenterYaw();
  if (next_phase == Phase::DirectExitDrive) {
    direct_exit_move_active_ = false;
  }
  if (!beginGridCenterAlign(current_grid_, target_yaw, next_phase,
                            "post_grab_center")) {
    return fail("post_grab_center_start_failed");
  }
  post_grab_center_next_phase_ = Phase::Done;
  writeBlackboardState("post_grab_center_align");
  if (node_) {
    RCLCPP_INFO(node_->get_logger(),
                "梅林预选赛夹取稳定后先归当前格中心：grid=%d yaw=%.3f，归位后进入%s",
                current_grid_, target_yaw, phaseText(next_phase));
  }
  return BT::NodeStatus::RUNNING;
}

double MfPreselectionFlowAction::postGrabCenterYaw() const {
  if (centerOdomReady()) {
    return normalizeAngle(center_odom_yaw_);
  }
  if (odomReady()) {
    return normalizeAngle(odom_yaw_);
  }
  return normalizeAngle(turn_target_yaw_);
}

BT::NodeStatus MfPreselectionFlowAction::tickCenterAlign() {
  if (!node_) {
    return fail("center_runtime_missing");
  }
  if ((node_->now() - phase_start_).seconds() >
      center_params_.align_timeout_s) {
    return fail("center_align_timeout_" + center_label_);
  }
  if (!centerOdomReady()) {
    center_stable_ticks_ = 0;
    publishCenterStop();
    if (!center_waiting_odom_logged_) {
      RCLCPP_WARN(node_->get_logger(),
                  "梅林预选赛格中心归位等待odom新鲜：%s，odom_topic=%s",
                  center_label_.c_str(), center_params_.odom_topic.c_str());
      center_waiting_odom_logged_ = true;
    }
    return BT::NodeStatus::RUNNING;
  }
  if (center_policy_ == StairCenterPolicy::EntryGrid2Reference &&
      !center_target_ready_) {
    if (!prepareEntryCenterTarget()) {
      return fail("entry_center_target_prepare_failed");
    }
  }

  const double error_x = center_target_x_ - center_odom_x_;
  const double error_y = center_target_y_ - center_odom_y_;
  const double distance = std::hypot(error_x, error_y);
  const double yaw_error = normalizeAngle(center_target_yaw_ - center_odom_yaw_);
  if (config().blackboard) {
    config().blackboard->set("mf_center_error_x", error_x);
    config().blackboard->set("mf_center_error_y", error_y);
    config().blackboard->set("mf_center_error_distance", distance);
  }

  const double yaw_tolerance_rad = center_params_.yaw_tolerance_deg * kDeg2Rad;
  if (distance <= center_params_.xy_tolerance_m &&
      std::abs(yaw_error) <= yaw_tolerance_rad) {
    ++center_stable_ticks_;
    publishCenterStop();
    if (center_stable_ticks_ >= center_params_.stable_ticks) {
      if (center_policy_ == StairCenterPolicy::EntryGrid2Reference) {
        writeCenterReference(2, center_target_x_, center_target_y_,
                             center_target_yaw_);
        current_grid_ = 2;
        config().blackboard->set("current_grid", current_grid_);
      }
      setCenterError("");
      RCLCPP_INFO(node_->get_logger(),
                  "梅林预选赛格中心归位完成：%s，target_grid=%d error=%.3fm yaw_error=%.3frad",
                  center_label_.c_str(), center_target_grid_, distance,
                  yaw_error);
      phase_ = center_next_phase_;
    }
    return BT::NodeStatus::RUNNING;
  }

  center_stable_ticks_ = 0;
  double body_vx = 0.0;
  double body_vy = 0.0;
  if (distance > center_params_.xy_tolerance_m) {
    const double world_vx = center_params_.xy_kp * error_x;
    const double world_vy = center_params_.xy_kp * error_y;
    const double c = std::cos(center_odom_yaw_);
    const double s = std::sin(center_odom_yaw_);
    body_vx = c * world_vx + s * world_vy;
    body_vy = -s * world_vx + c * world_vy;
    double body_speed = std::hypot(body_vx, body_vy);
    if (body_speed > center_params_.max_speed_mps && body_speed > 0.0) {
      const double scale = center_params_.max_speed_mps / body_speed;
      body_vx *= scale;
      body_vy *= scale;
      body_speed = center_params_.max_speed_mps;
    }
    if (body_speed < center_params_.min_speed_mps && body_speed > 1e-9) {
      const double scale = center_params_.min_speed_mps / body_speed;
      body_vx *= scale;
      body_vy *= scale;
    }
  }
  const double raw_wz = center_params_.yaw_kp * yaw_error;
  const double wz = std::clamp(raw_wz, -center_params_.yaw_max_speed_radps,
                               center_params_.yaw_max_speed_radps);
  publishCenterTwist(body_vx, body_vy, wz);
  return BT::NodeStatus::RUNNING;
}

bool MfPreselectionFlowAction::prepareEntryCenterTarget() {
  if (!centerOdomReady()) {
    return false;
  }
  center_target_x_ =
      center_odom_x_ +
      center_params_.entry_forward_offset_m * std::cos(center_target_yaw_);
  center_target_y_ =
      center_odom_y_ +
      center_params_.entry_forward_offset_m * std::sin(center_target_yaw_);
  center_target_ready_ = true;
  writeCenterTargetBlackboard();
  if (node_) {
    RCLCPP_INFO(node_->get_logger(),
                "梅林预选赛入口格中心目标已生成：x=%.3f y=%.3f yaw=%.3f",
                center_target_x_, center_target_y_, center_target_yaw_);
  }
  return true;
}

bool MfPreselectionFlowAction::readCenterReference(int &grid_id, double &x,
                                                   double &y,
                                                   double &yaw) const {
  if (!config().blackboard) {
    return false;
  }
  if (!config().blackboard->get("mf_center_reference_grid", grid_id) ||
      !config().blackboard->get("mf_center_reference_x", x) ||
      !config().blackboard->get("mf_center_reference_y", y) ||
      !config().blackboard->get("mf_center_reference_yaw", yaw)) {
    return false;
  }
  return validGrid(grid_id) && std::isfinite(x) && std::isfinite(y) &&
         std::isfinite(yaw);
}

void MfPreselectionFlowAction::writeCenterReference(int grid_id, double x,
                                                    double y, double yaw) {
  if (!config().blackboard) {
    return;
  }
  config().blackboard->set("mf_center_reference_grid", grid_id);
  config().blackboard->set("mf_center_reference_x", x);
  config().blackboard->set("mf_center_reference_y", y);
  config().blackboard->set("mf_center_reference_yaw", normalizeAngle(yaw));
  if (node_) {
    RCLCPP_INFO(node_->get_logger(),
                "梅林预选赛记录grid%d中心参考：x=%.3f y=%.3f yaw=%.3f",
                grid_id, x, y, normalizeAngle(yaw));
  }
}

bool MfPreselectionFlowAction::computeGridCenterFromReference(
    int target_grid, double &target_x, double &target_y) const {
  int reference_grid = 0;
  double reference_x = 0.0;
  double reference_y = 0.0;
  double reference_yaw = 0.0;
  if (!validGrid(target_grid) ||
      !readCenterReference(reference_grid, reference_x, reference_y,
                           reference_yaw)) {
    return false;
  }
  const int row_delta = gridRow(target_grid) - gridRow(reference_grid);
  const int col_delta = gridCol(target_grid) - gridCol(reference_grid);
  target_x = reference_x +
             static_cast<double>(row_delta) * center_params_.grid_step_m;
  target_y = reference_y -
             static_cast<double>(col_delta) * center_params_.grid_step_m;
  return std::isfinite(target_x) && std::isfinite(target_y);
}

void MfPreselectionFlowAction::writeCenterTargetBlackboard() const {
  if (!config().blackboard) {
    return;
  }
  config().blackboard->set("mf_center_target_grid", center_target_grid_);
  config().blackboard->set("mf_center_target_x", center_target_x_);
  config().blackboard->set("mf_center_target_y", center_target_y_);
}

void MfPreselectionFlowAction::setCenterError(
    const std::string &reason) const {
  if (!config().blackboard) {
    return;
  }
  config().blackboard->set("mf_center_error", reason);
  config().blackboard->set("mf_transition_error", reason);
}

bool MfPreselectionFlowAction::prepareTransitionTo(int target_grid) {
  // 格间转换仍以 MerlinMapManager 的静态深度表为准，只允许相邻格上/下
  // 一档高度。平地同高移动和跨多档高度都不是本预选赛链路的合法动作。
  std::shared_ptr<MerlinMapManager> map;
  if (!config().blackboard->get("merlin_map", map) || !map) {
    return false;
  }
  const int from_depth = map->getDepth(current_grid_);
  const int to_depth = map->getDepth(target_grid);
  if (from_depth < 0 || to_depth < 0) {
    return false;
  }
  transition_from_grid_ = current_grid_;
  transition_target_grid_ = target_grid;
  transition_height_delta_ = to_depth - from_depth;
  if (std::abs(transition_height_delta_) != 1) {
    return false;
  }
  // transition_high_side_ 后续用于决定 TransitionObserve 阶段发现 R2 KFS 时
  // 该用上侧夹取命令还是下侧夹取命令。
  transition_high_side_ = transition_height_delta_ > 0;
  return true;
}

BT::NodeStatus MfPreselectionFlowAction::startTransitionTo(int target_grid) {
  // 启动格间转换时先转到台阶动作所需 yaw，再进入机械臂高低侧调整。
  if (!prepareTransitionTo(target_grid)) {
    return fail("invalid_transition");
  }
  const double yaw = transitionYaw(transition_from_grid_, transition_target_grid_,
                                   transition_height_delta_);
  RCLCPP_INFO(node_->get_logger(),
              "梅林预选赛准备格间转换：grid%d -> grid%d，高度差=%d，目标yaw=%.3f",
              transition_from_grid_, transition_target_grid_,
              transition_height_delta_, yaw);
  beginTurnYaw(yaw, Phase::TransitionArmAdjust, "transition_turn");
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus MfPreselectionFlowAction::startFakeAvoidForwardObservation() {
  fake_avoid_forward_mode_ = true;
  if (current_grid_ == 10 || current_grid_ == 12) {
    RCLCPP_INFO(node_->get_logger(),
                "梅林预选赛假KFS旁列推进到出口行grid%d，准备下阶梯离场",
                current_grid_);
    phase_ = Phase::FinalExitYawAlign;
    return BT::NodeStatus::RUNNING;
  }
  const auto target_grid =
      MfPreselectionLogicResult::fakeAvoidanceForwardTargetGrid(current_grid_);
  if (!target_grid.has_value()) {
    return fail("invalid_fake_avoid_forward_target");
  }
  RCLCPP_INFO(node_->get_logger(),
              "梅林预选赛假KFS旁列前向观察：grid%d -> grid%d",
              current_grid_, *target_grid);
  return startFakeAvoidForwardTransitionTo(*target_grid);
}

BT::NodeStatus
MfPreselectionFlowAction::startFakeAvoidForwardTransitionTo(int target_grid) {
  // 假 KFS 旁列推进要先朝目标格正前方观察 R2 KFS；如果这条边是下阶，
  // 台阶原语需要的后轮先下 yaw 会在 TransitionStair 前再单独对齐。
  if (!prepareTransitionTo(target_grid)) {
    return fail("invalid_transition");
  }
  const double observe_yaw =
      transitionEdgeYaw(transition_from_grid_, transition_target_grid_);
  const double stair_yaw = transitionYaw(transition_from_grid_,
                                         transition_target_grid_,
                                         transition_height_delta_);
  RCLCPP_INFO(
      node_->get_logger(),
      "梅林预选赛假KFS旁列准备前向观察：grid%d -> grid%d，高度差=%d，observe_yaw=%.3f stair_yaw=%.3f",
      transition_from_grid_, transition_target_grid_, transition_height_delta_,
      observe_yaw, stair_yaw);
  beginTurnYaw(observe_yaw, Phase::TransitionArmAdjust,
               "fake_avoid_forward_observe_turn");
  return BT::NodeStatus::RUNNING;
}

bool MfPreselectionFlowAction::continueAfterTransition() {
  // current_grid 只在台阶原语完整成功后提交，避免中途失败时黑板误认为
  // 机器人已经到达目标格。
  current_grid_ = transition_target_grid_;
  config().blackboard->set("current_grid", current_grid_);
  writeBlackboardState("transition_done");
  RCLCPP_INFO(node_->get_logger(),
              "梅林预选赛格间转换完成：当前grid=%d，直出模式=%s，假KFS旁列推进=%s",
              current_grid_, direct_exit_mode_ ? "是" : "否",
              fake_avoid_forward_mode_ ? "是" : "否");
  const Phase next_phase = phaseAfterTransition();
  if (!beginGridCenterAlign(current_grid_, turn_target_yaw_, next_phase,
                            "grid_transition_center")) {
    return false;
  }
  return true;
}

MfPreselectionFlowAction::Phase
MfPreselectionFlowAction::phaseAfterTransition() const {
  if (fake_avoid_forward_mode_) {
    if (current_grid_ == 10 || current_grid_ == 12) {
      return Phase::FinalExitYawAlign;
    }
    return Phase::FakeAvoidForwardStep;
  }
  if (direct_exit_mode_) {
    // 直出模式回到 AfterEntry 统一调度，让它仍能在下一格做前方守卫检测。
    return Phase::AfterEntry;
  }
  if (current_grid_ == 5 || current_grid_ == 8) {
    return Phase::RowScanTurnLeft;
  }
  if (current_grid_ == 11) {
    return Phase::Row4ForcedTurn;
  }
  if (current_grid_ == 12) {
    return Phase::FinalExitYawAlign;
  }
  return Phase::RowFrontDetect;
}

double MfPreselectionFlowAction::transitionEdgeYaw(int from_grid,
                                                   int target_grid) const {
  // 离散格方向约定与 MF 主链一致：行号增加是 +X，列号减少是 +Y。
  const int row_delta = gridRow(target_grid) - gridRow(from_grid);
  const int col_delta = gridCol(target_grid) - gridCol(from_grid);
  const double dx = static_cast<double>(row_delta);
  const double dy = static_cast<double>(-col_delta);
  return std::atan2(dy, dx);
}

double MfPreselectionFlowAction::transitionYaw(int from_grid, int target_grid,
                                               int height_delta) const {
  // 上台阶面向目标边，下台阶反向，让后轮先下。
  const double edge_yaw = transitionEdgeYaw(from_grid, target_grid);
  return normalizeAngle(height_delta > 0 ? edge_yaw : edge_yaw + kPi);
}

void MfPreselectionFlowAction::beginStair(StairMode mode, Phase next_phase,
                                          std::string label,
                                          StairCenterPolicy center_policy) {
  // 台阶动作在本节点内部复刻独立 StairClimb/StairDescend 的关键时序：
  // 推杆命令通过 service，轮组到位通过激光事件；跨阶梯前先完成 yaw 对齐，
  // 直行阶段只做小幅 heading hold，偏差超 gate 时停车原地修正。
  stair_mode_ = mode;
  stair_next_phase_ = next_phase;
  stair_center_policy_ = center_policy;
  stair_label_ = std::move(label);
  stair_after_heading_phase_ = (mode == StairMode::Climb)
                                   ? StairPhase::ClimbSendFrontExtend
                                   : StairPhase::DescendDriveUntilRearEvent;
  stair_phase_ = StairPhase::HeadingAlign;
  active_wheel_event_label_.clear();
  active_wheel_event_started_ = false;
  timed_drive_started_ = false;
  stair_drive_profile_started_ = false;
  stair_heading_stable_ticks_ = 0;
  stair_heading_waiting_odom_logged_ = false;
  stair_heading_gate_logged_ = false;
  if (node_) {
    phase_start_ = node_->now();
    RCLCPP_INFO(node_->get_logger(),
                "梅林预选赛开始%s动作：%s，先对齐yaw=%.3frad，完成后进入下一阶段",
                stairModeText(mode), stair_label_.c_str(), turn_target_yaw_);
  }
  phase_ = Phase::StairPrimitive;
}

BT::NodeStatus MfPreselectionFlowAction::tickStair() {
  switch (stair_phase_) {
  case StairPhase::HeadingAlign:
    return tickStairHeadingAlign();
  case StairPhase::ClimbSendFrontExtend:
    // 上台阶第 1 步：伸出前推杆，只要求 transport ACK。
    beginMechanismCommand(static_cast<uint8_t>(rc26_serial::CommandID::FRONT_PUSHROD_EXTEND),
                          "FRONT_PUSHROD_EXTEND", -1, Phase::StairPrimitive,
                          "front_pushrod_extend_failed");
    break;
  case StairPhase::ClimbHoldAfterFrontExtend:
    // 给前推杆伸出留机械稳定时间，等待期间持续零速。
    RCLCPP_INFO(node_->get_logger(),
                "梅林预选赛上阶梯：前推杆伸出ACK完成，零速等待 %.2fs",
                stair_params_.climb_front_extend_delay_s);
    stair_phase_ = StairPhase::ClimbDriveUntilFrontFirstEvent;
    beginZeroHold(stair_params_.climb_front_extend_delay_s,
                  Phase::StairPrimitive, "climb_front_extend_hold");
    break;
  case StairPhase::ClimbDriveUntilFrontFirstEvent:
    // 上台阶前轮阶段：x 正方向前进，直到前轮第一激光高度突变 0x04。
    if (guardPathObstacles()) {
      phase_start_ = node_->now();
      break;
    }
    if (tickStairDriveYawGate("climb_front_first")) {
      break;
    }
    if (!active_wheel_event_started_) {
      beginWheelEvent(WheelEvent::FrontFirst, stair_params_.front_event_timeout_s,
                      "front_first");
      beginStairDriveProfile(stair_params_.climb_front_drive_profile,
                             "climb_front_first");
    }
    publishProfiledStairTwist(1.0);
    if (const auto wheel_status = tickWheelEvent();
        wheel_status == BT::NodeStatus::SUCCESS) {
      publishStop();
      RCLCPP_INFO(node_->get_logger(),
                  "梅林预选赛上阶梯：前轮第一激光事件确认，准备前推杆收回+后推杆伸出");
      stair_phase_ = StairPhase::ClimbSendFrontRetractAndRearExtend;
      active_wheel_event_label_.clear();
      active_wheel_event_started_ = false;
    } else if (wheel_status == BT::NodeStatus::FAILURE) {
      return BT::NodeStatus::FAILURE;
    }
    break;
  case StairPhase::ClimbSendFrontRetractAndRearExtend:
    // 前轮已上台阶后，同时收回前推杆并伸出后推杆，为后轮上台阶做支撑。
    beginCommandPair(static_cast<uint8_t>(rc26_serial::CommandID::FRONT_PUSHROD_RETRACT),
                     "FRONT_PUSHROD_RETRACT",
                     static_cast<uint8_t>(rc26_serial::CommandID::REAR_PUSHROD_EXTEND),
                     "REAR_PUSHROD_EXTEND", Phase::StairPrimitive,
                     "climb_pair_command_failed");
    break;
  case StairPhase::ClimbHoldAfterFrontRetractAndRearExtend:
    RCLCPP_INFO(node_->get_logger(),
                "梅林预选赛上阶梯：前推杆收回+后推杆伸出ACK完成，零速等待 %.2fs",
                stair_params_.climb_retract_rear_extend_delay_s);
    stair_phase_ = StairPhase::ClimbDriveUntilRearEvent;
    beginZeroHold(stair_params_.climb_retract_rear_extend_delay_s,
                  Phase::StairPrimitive, "climb_pair_hold");
    break;
  case StairPhase::ClimbDriveUntilRearEvent:
    // 上台阶后轮阶段：速度使用 fast->slow profile，尾段减速以降低冲击；
    // 直到后轮激光高度突变 0x05。
    if (guardPathObstacles()) {
      phase_start_ = node_->now();
      break;
    }
    if (tickStairDriveYawGate("climb_rear")) {
      break;
    }
    if (!active_wheel_event_started_) {
      beginWheelEvent(WheelEvent::Rear, stair_params_.rear_event_timeout_s,
                      "rear");
      beginStairDriveProfile(stair_params_.climb_rear_drive_profile,
                             "climb_rear");
    }
    publishProfiledStairTwist(1.0);
    if (const auto wheel_status = tickWheelEvent();
        wheel_status == BT::NodeStatus::SUCCESS) {
      publishStop();
      RCLCPP_INFO(node_->get_logger(),
                  "梅林预选赛上阶梯：后轮激光事件确认，准备收回后推杆");
      stair_phase_ = StairPhase::ClimbSendRearRetract;
      active_wheel_event_label_.clear();
      active_wheel_event_started_ = false;
    } else if (wheel_status == BT::NodeStatus::FAILURE) {
      return BT::NodeStatus::FAILURE;
    }
    break;
  case StairPhase::ClimbSendRearRetract:
    // 后轮到位后收回后推杆，完成上台阶机构复位。
    beginMechanismCommand(static_cast<uint8_t>(rc26_serial::CommandID::REAR_PUSHROD_RETRACT),
                          "REAR_PUSHROD_RETRACT", -1, Phase::StairPrimitive,
                          "rear_pushrod_retract_failed");
    break;
  case StairPhase::ClimbHoldAfterRearRetract:
    RCLCPP_INFO(node_->get_logger(),
                "梅林预选赛上阶梯：后推杆收回ACK完成，零速等待 %.2fs 后完成上阶梯",
                stair_params_.climb_rear_retract_delay_s);
    stair_phase_ = StairPhase::Complete;
    beginZeroHold(stair_params_.climb_rear_retract_delay_s,
                  Phase::StairPrimitive, "climb_rear_retract_hold");
    break;
  case StairPhase::DescendDriveUntilRearEvent:
    // 下台阶第 1 步：x 负方向后退，先等后轮激光高度突变 0x05。
    if (guardPathObstacles()) {
      phase_start_ = node_->now();
      break;
    }
    if (tickStairDriveYawGate("descend_rear")) {
      break;
    }
    publishTwist(-stair_params_.descend_rear_drive_speed_mps, 0.0,
                 stairHeadingAngularZ());
    if (!active_wheel_event_started_) {
      beginWheelEvent(WheelEvent::Rear, stair_params_.rear_event_timeout_s,
                      "rear");
    }
    if (const auto wheel_status = tickWheelEvent();
        wheel_status == BT::NodeStatus::SUCCESS) {
      publishStop();
      RCLCPP_INFO(node_->get_logger(),
                  "梅林预选赛下阶梯：后轮激光事件确认，准备伸出后推杆");
      stair_phase_ = StairPhase::DescendSendRearExtend;
      active_wheel_event_label_.clear();
      active_wheel_event_started_ = false;
    } else if (wheel_status == BT::NodeStatus::FAILURE) {
      return BT::NodeStatus::FAILURE;
    }
    break;
  case StairPhase::DescendSendRearExtend:
    // 后轮下台阶触发后伸出后推杆，准备控制车身继续下阶。
    beginMechanismCommand(static_cast<uint8_t>(rc26_serial::CommandID::REAR_PUSHROD_EXTEND),
                          "REAR_PUSHROD_EXTEND", -1, Phase::StairPrimitive,
                          "rear_pushrod_extend_failed");
    break;
  case StairPhase::DescendHoldAfterRearExtend:
    RCLCPP_INFO(node_->get_logger(),
                "梅林预选赛下阶梯：后推杆伸出ACK完成，零速等待 %.2fs",
                stair_params_.descend_rear_extend_delay_s);
    stair_phase_ = StairPhase::DescendDriveUntilFrontSecondEvent;
    beginZeroHold(stair_params_.descend_rear_extend_delay_s,
                  Phase::StairPrimitive, "descend_rear_extend_hold");
    break;
  case StairPhase::DescendDriveUntilFrontSecondEvent:
    // 下台阶前轮阶段只等待前轮第二激光 0x07，不使用 0x04 作为推进条件。
    if (guardPathObstacles()) {
      phase_start_ = node_->now();
      break;
    }
    if (tickStairDriveYawGate("descend_front_second")) {
      break;
    }
    if (!active_wheel_event_started_) {
      beginWheelEvent(WheelEvent::FrontSecond,
                      stair_params_.front_event_timeout_s, "front_second");
      beginStairDriveProfile(stair_params_.descend_front_second_drive_profile,
                             "descend_front_second");
    }
    publishProfiledStairTwist(-1.0);
    if (const auto wheel_status = tickWheelEvent();
        wheel_status == BT::NodeStatus::SUCCESS) {
      publishStop();
      RCLCPP_INFO(node_->get_logger(),
                  "梅林预选赛下阶梯：前轮第二激光事件确认，准备后推杆收回+前推杆伸出");
      stair_phase_ = StairPhase::DescendSendRearRetractAndFrontExtend;
      active_wheel_event_label_.clear();
      active_wheel_event_started_ = false;
    } else if (wheel_status == BT::NodeStatus::FAILURE) {
      return BT::NodeStatus::FAILURE;
    }
    break;
  case StairPhase::DescendSendRearRetractAndFrontExtend:
    // 前轮第二激光触发后，同时收回后推杆并伸出前推杆，进入前推杆收回前
    // 的定时后退段。
    beginCommandPair(static_cast<uint8_t>(rc26_serial::CommandID::REAR_PUSHROD_RETRACT),
                     "REAR_PUSHROD_RETRACT",
                     static_cast<uint8_t>(rc26_serial::CommandID::FRONT_PUSHROD_EXTEND),
                     "FRONT_PUSHROD_EXTEND", Phase::StairPrimitive,
                     "descend_pair_command_failed");
    break;
  case StairPhase::DescendHoldAfterRearRetractAndFrontExtend:
    RCLCPP_INFO(node_->get_logger(),
                "梅林预选赛下阶梯：后推杆收回+前推杆伸出ACK完成，零速等待 %.2fs",
                stair_params_.descend_retract_front_extend_delay_s);
    stair_phase_ = StairPhase::DescendTimedDriveBeforeFrontRetract;
    beginZeroHold(stair_params_.descend_retract_front_extend_delay_s,
                  Phase::StairPrimitive, "descend_pair_hold");
    break;
  case StairPhase::DescendTimedDriveBeforeFrontRetract:
    // 定时后退用于给前推杆收回创造空间；这段没有额外激光事件判定，
    // 只由参数时长控制。
    if (tickStairDriveYawGate("descend_front_retract_timed")) {
      break;
    }
    if (!timed_drive_started_) {
      phase_start_ = node_->now();
      timed_drive_started_ = true;
      RCLCPP_INFO(node_->get_logger(),
                  "梅林预选赛下阶梯：前推杆收回前定时后退，speed=%.3fm/s duration=%.2fs",
                  stair_params_.descend_front_retract_timed_drive_speed_mps,
                  stair_params_.descend_front_retract_drive_duration_s);
    }
    if ((node_->now() - phase_start_).seconds() >=
        stair_params_.descend_front_retract_drive_duration_s) {
      publishStop();
      RCLCPP_INFO(node_->get_logger(),
                  "梅林预选赛下阶梯：前推杆收回前定时后退完成，准备收回前推杆");
      stair_phase_ = StairPhase::DescendSendFrontRetract;
    } else {
      publishTwist(-stair_params_.descend_front_retract_timed_drive_speed_mps,
                   0.0, stairHeadingAngularZ());
    }
    break;
  case StairPhase::DescendSendFrontRetract:
    // 定时后退完成后收回前推杆，准备结束下台阶。
    beginMechanismCommand(static_cast<uint8_t>(rc26_serial::CommandID::FRONT_PUSHROD_RETRACT),
                          "FRONT_PUSHROD_RETRACT", -1, Phase::StairPrimitive,
                          "front_pushrod_retract_failed");
    break;
  case StairPhase::DescendHoldAfterFrontRetract:
    RCLCPP_INFO(node_->get_logger(),
                "梅林预选赛下阶梯：前推杆收回ACK完成，零速等待 %.2fs 后完成下阶梯",
                stair_params_.descend_front_retract_delay_s);
    stair_phase_ = StairPhase::Complete;
    beginZeroHold(stair_params_.descend_front_retract_delay_s,
                  Phase::StairPrimitive, "descend_front_retract_hold");
    break;
  case StairPhase::Complete:
    // 对格间转换，台阶完成后才提交 current_grid；对入口/离场独立台阶，
    // 直接进入 beginStair() 指定的下一 phase。
    RCLCPP_INFO(node_->get_logger(), "梅林预选赛台阶动作完成：%s",
                stair_label_.c_str());
    if (stair_center_policy_ == StairCenterPolicy::EntryGrid2Reference) {
      if (!beginEntryCenterAdvance(stair_next_phase_, "entry_grid2_center")) {
        return fail("entry_center_start_failed");
      }
    } else if (stair_center_policy_ ==
               StairCenterPolicy::TransitionTargetGrid) {
      if (!continueAfterTransition()) {
        return fail("transition_center_start_failed");
      }
    } else if (stair_center_policy_ ==
               StairCenterPolicy::FakeAvoidTargetGrid) {
      current_grid_ = fake_avoid_target_grid_;
      config().blackboard->set("current_grid", current_grid_);
      writeBlackboardState("fake_avoid_transition_done");
      if (!beginGridCenterAlign(current_grid_, turn_target_yaw_,
                                stair_next_phase_, "fake_avoid_center")) {
        return fail("fake_avoid_center_start_failed");
      }
    } else if (stair_center_policy_ ==
               StairCenterPolicy::FinalExitVirtual) {
      if (!beginFinalExitCenterAlign(stair_next_phase_,
                                     "final_exit_virtual_center")) {
        return fail("final_exit_center_start_failed");
      }
    } else {
      phase_ = stair_next_phase_;
    }
    stair_center_policy_ = StairCenterPolicy::None;
    break;
  }
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus MfPreselectionFlowAction::tickStairHeadingAlign() {
  if (!stair_params_.heading_hold_enable) {
    stair_phase_ = stair_after_heading_phase_;
    return BT::NodeStatus::RUNNING;
  }
  if (!node_) {
    return fail("stair_heading_runtime_missing");
  }
  if ((node_->now() - phase_start_).seconds() >
      stair_params_.heading_align_timeout_s) {
    publishStop();
    return fail("stair_heading_align_timeout");
  }
  if (!odomReady()) {
    publishStop();
    if (!stair_heading_waiting_odom_logged_) {
      stair_heading_waiting_odom_logged_ = true;
      RCLCPP_WARN(node_->get_logger(),
                  "梅林预选赛台阶yaw预对齐等待odom新鲜：%s，odom_topic=%s",
                  stair_label_.c_str(), params_.odom_topic.c_str());
    }
    return BT::NodeStatus::RUNNING;
  }

  const double error =
      normalizeAngle(turn_target_yaw_ - odom_yaw_);
  const double tolerance_rad =
      std::abs(stair_params_.heading_tolerance_deg) * kDeg2Rad;
  if (std::abs(error) <= tolerance_rad) {
    ++stair_heading_stable_ticks_;
    publishStop();
    if (stair_heading_stable_ticks_ >= stair_params_.heading_stable_ticks) {
      RCLCPP_INFO(node_->get_logger(),
                  "梅林预选赛台阶yaw预对齐完成：%s current=%.3f target=%.3f error=%.3frad",
                  stair_label_.c_str(), odom_yaw_, turn_target_yaw_, error);
      stair_phase_ = stair_after_heading_phase_;
      stair_heading_gate_logged_ = false;
    }
    return BT::NodeStatus::RUNNING;
  }

  stair_heading_stable_ticks_ = 0;
  publishTwist(0.0, 0.0, stairHeadingAngularZ());
  return BT::NodeStatus::RUNNING;
}

void MfPreselectionFlowAction::beginWheelEvent(WheelEvent event,
                                               double timeout_s,
                                               std::string label) {
  // 进入激光等待时记录三类事件计数 baseline；后续只看计数是否增长，
  // 不要求反馈消息恰好在同一个 tick 到达。
  active_wheel_event_ = event;
  active_wheel_event_timeout_s_ = std::max(kMinTimeoutS, timeout_s);
  active_wheel_event_label_ = std::move(label);
  active_wheel_event_started_ = true;
  front_first_event_baseline_ =
      front_first_event_count_.load(std::memory_order_relaxed);
  front_second_event_baseline_ =
      front_second_event_count_.load(std::memory_order_relaxed);
  rear_event_baseline_ = rear_event_count_.load(std::memory_order_relaxed);
  if (node_) {
    phase_start_ = node_->now();
    RCLCPP_INFO(node_->get_logger(),
                "梅林预选赛开始等待激光事件：%s，阶段=%s，timeout=%.2fs",
                wheelEventText(event), active_wheel_event_label_.c_str(),
                active_wheel_event_timeout_s_);
  }
}

bool MfPreselectionFlowAction::wheelEventReceived() const {
  // feedback_sub_ 按反馈 ID 分别累加计数。这里用 baseline 差值判断“本阶段之后”
  // 是否收到过目标事件，避免启动前残留事件误触发。
  switch (active_wheel_event_) {
  case WheelEvent::FrontFirst:
    return front_first_event_count_.load(std::memory_order_relaxed) >
           front_first_event_baseline_;
  case WheelEvent::FrontSecond:
    return front_second_event_count_.load(std::memory_order_relaxed) >
           front_second_event_baseline_;
  case WheelEvent::Rear:
    return rear_event_count_.load(std::memory_order_relaxed) >
           rear_event_baseline_;
  }
  return false;
}

BT::NodeStatus MfPreselectionFlowAction::tickWheelEvent() {
  // 激光事件是台阶动作的安全推进条件；超时统一走 fail()，先停车再返回 FAILURE。
  if (wheelEventReceived()) {
    RCLCPP_INFO(node_->get_logger(),
                "梅林预选赛收到激光事件：%s，阶段=%s",
                wheelEventText(active_wheel_event_),
                active_wheel_event_label_.c_str());
    return BT::NodeStatus::SUCCESS;
  }
  if (node_ && (node_->now() - phase_start_).seconds() >
                   active_wheel_event_timeout_s_) {
    if (node_) {
      RCLCPP_WARN(node_->get_logger(),
                  "梅林预选赛等待激光事件超时：%s，阶段=%s",
                  wheelEventText(active_wheel_event_),
                  active_wheel_event_label_.c_str());
    }
    phase_ = Phase::Done;
    (void)fail("wheel_event_timeout");
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::RUNNING;
}

void MfPreselectionFlowAction::beginStairDriveProfile(
    const StairSpeedProfile &profile, std::string label) {
  stair_drive_profile_ = normalizeStairSpeedProfile(profile);
  stair_drive_profile_label_ = std::move(label);
  stair_drive_profile_started_ = false;
  has_last_cmd_publish_ = false;
  if (node_) {
    RCLCPP_INFO(node_->get_logger(),
                "梅林预选赛台阶速度规划启动：%s %.3f->%.3fm/s %.2fs",
                stair_drive_profile_label_.c_str(),
                stair_drive_profile_.fast_speed_mps,
                stair_drive_profile_.slow_speed_mps,
                stair_drive_profile_.slowdown_duration_s);
  }
}

double MfPreselectionFlowAction::stairDriveProfileSpeed() {
  if (!node_) {
    return stair_drive_profile_.fast_speed_mps;
  }
  if (!stair_drive_profile_started_) {
    stair_drive_profile_start_ = node_->now();
    stair_drive_profile_started_ = true;
  }
  return sampleStairSpeedProfile(
      stair_drive_profile_,
      (node_->now() - stair_drive_profile_start_).seconds());
}

double MfPreselectionFlowAction::stairHeadingAngularZ() const {
  if (!stair_params_.heading_hold_enable || !odomReady()) {
    return 0.0;
  }
  const double error = normalizeAngle(turn_target_yaw_ - odom_yaw_);
  const double raw = stair_params_.heading_kp * error;
  const double limit = std::abs(stair_params_.heading_max_speed_radps);
  return std::clamp(raw, -limit, limit);
}

bool MfPreselectionFlowAction::stairHeadingReady() const {
  if (!stair_params_.heading_hold_enable) {
    return true;
  }
  if (!odomReady()) {
    return false;
  }
  const double gate_rad =
      std::abs(stair_params_.heading_gate_deg) * kDeg2Rad;
  const double error = normalizeAngle(turn_target_yaw_ - odom_yaw_);
  return std::abs(error) <= gate_rad;
}

bool MfPreselectionFlowAction::tickStairDriveYawGate(
    const std::string &label) {
  if (!stair_params_.heading_hold_enable) {
    return false;
  }
  if (!odomReady()) {
    publishStop();
    phase_start_ = node_->now();
    has_last_cmd_publish_ = false;
    if (!stair_heading_waiting_odom_logged_) {
      stair_heading_waiting_odom_logged_ = true;
      RCLCPP_WARN(node_->get_logger(),
                  "梅林预选赛台阶直行等待odom新鲜：%s/%s，odom_topic=%s",
                  stair_label_.c_str(), label.c_str(), params_.odom_topic.c_str());
    }
    return true;
  }
  stair_heading_waiting_odom_logged_ = false;
  if (stairHeadingReady()) {
    stair_heading_gate_logged_ = false;
    return false;
  }

  publishTwist(0.0, 0.0, stairHeadingAngularZ());
  phase_start_ = node_->now();
  has_last_cmd_publish_ = false;
  if (stair_drive_profile_started_) {
    stair_drive_profile_start_ = node_->now();
  }
  if (!stair_heading_gate_logged_) {
    const double error = normalizeAngle(turn_target_yaw_ - odom_yaw_);
    RCLCPP_WARN(node_->get_logger(),
                "梅林预选赛台阶直行yaw超gate，暂停线速度先纠偏：%s/%s current=%.3f target=%.3f error=%.3frad gate=%.1fdeg",
                stair_label_.c_str(), label.c_str(), odom_yaw_, turn_target_yaw_,
                error, stair_params_.heading_gate_deg);
    stair_heading_gate_logged_ = true;
  }
  return true;
}

void MfPreselectionFlowAction::publishProfiledStairTwist(
    double direction_sign) {
  publishTwist((direction_sign < 0.0 ? -1.0 : 1.0) *
                   stairDriveProfileSpeed(),
               0.0, stairHeadingAngularZ());
}

void MfPreselectionFlowAction::recordR2LockReject(
    const std::string &reason, const std::string &detail, int64_t sequence) {
  last_r2_lock_reject_reason_ = reason;
  last_r2_lock_reject_detail_ = detail;
  last_r2_lock_reject_sequence_ = sequence;

  if (config().blackboard) {
    config().blackboard->set("mf_preselect_r2_lock_reject_reason",
                             last_r2_lock_reject_reason_);
    config().blackboard->set("mf_preselect_r2_lock_reject_detail",
                             last_r2_lock_reject_detail_);
    config().blackboard->set("mf_preselect_r2_lock_reject_sequence",
                             last_r2_lock_reject_sequence_);
  }

  if (node_ &&
      r2_lock_logged_reasons_this_detection_.insert(reason).second) {
    const std::string reason_text =
        translateR2LockRejectReason(last_r2_lock_reject_reason_);
    RCLCPP_INFO(
        node_->get_logger(),
        "梅林预选赛R2 KFS候选拒绝：原因=%s，原因代码=%s，详情={%s}",
        reason_text.c_str(),
        last_r2_lock_reject_reason_.c_str(),
        last_r2_lock_reject_detail_.c_str());
  }
}

void MfPreselectionFlowAction::clearR2LockReject() {
  last_r2_lock_reject_reason_.clear();
  last_r2_lock_reject_detail_.clear();
  last_r2_lock_reject_sequence_ = 0;
  if (config().blackboard) {
    config().blackboard->set("mf_preselect_r2_lock_reject_reason",
                             last_r2_lock_reject_reason_);
    config().blackboard->set("mf_preselect_r2_lock_reject_detail",
                             last_r2_lock_reject_detail_);
    config().blackboard->set("mf_preselect_r2_lock_reject_sequence",
                             last_r2_lock_reject_sequence_);
  }
}

std::string MfPreselectionFlowAction::r2LockRejectSummary() const {
  if (last_r2_lock_reject_reason_.empty()) {
    return "";
  }
  return "，最近R2候选拒绝=" +
         translateR2LockRejectReason(last_r2_lock_reject_reason_) +
         "，原因代码=" + last_r2_lock_reject_reason_ +
         "，详情={" + last_r2_lock_reject_detail_ + "}";
}

std::optional<MfPreselectionFlowAction::KfsVisualObservation>
MfPreselectionFlowAction::findR2LockObservation(
    R2DepthProfile depth_profile, R2LockObservationMode mode) {
  const bool allow_depthless_align =
      mode == R2LockObservationMode::AllowDepthlessForAlign;
  const auto profileText = [](R2DepthProfile profile) {
    return profile == R2DepthProfile::Entry ? "入口深度窗口" : "常规深度窗口";
  };
  const auto detailPrefix = [this, depth_profile,
                             &profileText](int64_t sequence) {
    const bool entry_profile = depth_profile == R2DepthProfile::Entry;
    const double min_depth =
        entry_profile ? params_.entry_depth_min_m : params_.depth_min_m;
    const double max_depth =
        entry_profile ? params_.entry_depth_max_m : params_.depth_max_m;
    std::ostringstream oss;
    oss << "检测模式=" << detectModeText(detect_mode_) << " 当前格="
        << current_grid_ << " 图像序号=" << sequence << " 深度档位="
        << profileText(depth_profile) << " 深度窗口=[" << min_depth << ","
        << max_depth << "] 已夹取数量=" << pickup_count_ << "/"
        << params_.max_pickup_count;
    return oss.str();
  };

  if (!canPickup()) {
    std::ostringstream detail;
    detail << detailPrefix(0) << " can_pickup=否";
    recordR2LockReject("pickup_limit_reached", detail.str(), 0);
    return std::nullopt;
  }
  if (!vision_ || !vision_->isRunning()) {
    std::ostringstream detail;
    detail << detailPrefix(0)
           << " 视觉实例=" << yesNoText(static_cast<bool>(vision_))
           << " 视觉运行=" << yesNoText(vision_ && vision_->isRunning());
    recordR2LockReject("vision_not_running", detail.str(), 0);
    return std::nullopt;
  }

  rc26_vision::VisionInferenceManager::FrameSnapshot snapshot;
  const bool got_snapshot = vision_->getLatestFrameSnapshot(snapshot);
  if (!got_snapshot || !snapshot.has_display || !snapshot.has_color ||
      snapshot.color_bgr.empty() || !snapshot.has_depth || snapshot.depth.empty() ||
      snapshot.display_sequence <= 0) {
    std::ostringstream detail;
    detail << detailPrefix(got_snapshot ? snapshot.display_sequence : 0)
           << " 获取快照=" << yesNoText(got_snapshot)
           << " 有显示图=" << yesNoText(snapshot.has_display)
           << " 有彩色图=" << yesNoText(snapshot.has_color)
           << " 彩色图为空=" << yesNoText(snapshot.color_bgr.empty())
           << " 有深度图=" << yesNoText(snapshot.has_depth)
           << " 深度图为空=" << yesNoText(snapshot.depth.empty())
           << " 彩色尺寸=" << snapshot.color_bgr.cols << "x"
           << snapshot.color_bgr.rows << " 深度尺寸=" << snapshot.depth.cols
           << "x" << snapshot.depth.rows
           << " 深度类型=" << matTypeName(snapshot.depth)
           << " 检测框总数=" << snapshot.detections.size();
    recordR2LockReject("snapshot_invalid", detail.str(),
                       got_snapshot ? snapshot.display_sequence : 0);
    return std::nullopt;
  }
  if (snapshot.display_sequence == kfs_align_target_lock_sequence_) {
    return kfs_align_last_observation_;
  }

  kfs_align_target_lock_sequence_ = snapshot.display_sequence;
  kfs_align_last_observation_.reset();

  struct CandidateDepthState {
    bool has_depth{false};
    double depth_m{0.0};
    MfPreselectionLogicResult::KfsDepthSource source{
        MfPreselectionLogicResult::KfsDepthSource::None};
    std::string detail;
  };

  const bool entry_profile = depth_profile == R2DepthProfile::Entry;
  rc26_vision::DepthRoiSamplerConfig depth_config;
  depth_config.roi_size = params_.kfs_depth_roi_size;
  depth_config.min_valid_count = params_.kfs_depth_min_valid_count;
  depth_config.min_depth_m =
      entry_profile ? params_.entry_depth_min_m : params_.depth_min_m;
  depth_config.max_depth_m =
      entry_profile ? params_.entry_depth_max_m : params_.depth_max_m;

  std::vector<rc26_vision::Detection> candidates;
  candidates.reserve(snapshot.detections.size());
  std::vector<MfPreselectionTargetSnapshot> snapshots;
  snapshots.reserve(snapshot.detections.size());
  std::vector<CandidateDepthState> depth_states;
  depth_states.reserve(snapshot.detections.size());
  int label_match_count = 0;
  int ignored_count = 0;
  int depth_invalid_count = 0;
  int usable_depth_count = 0;
  int mono_depth_count = 0;
  int depthless_align_count = 0;
  std::string ignored_detail;
  std::string depth_invalid_detail;
  int depth_invalid_best_window_valid_count = -1;
  int depth_invalid_best_raw_valid_count = -1;
  std::optional<double> first_valid_depth;
  std::string first_valid_depth_detail;

  for (const auto &det : snapshot.detections) {
    const std::string name = rc26_vision::visualTargetLabel(det);
    if (!MfPreselectionLogicResult::labelMatches(
            name, params_.r2_target_labels, params_.r2_target_label_prefixes)) {
      continue;
    }
    ++label_match_count;
    const MfPreselectionTargetSnapshot candidate =
        rc26_vision::makeVisualTargetSnapshot(det, snapshot.display_sequence);
    double ignored_best_iou = 0.0;
    for (const auto &ignored : ignored_r2_targets_) {
      ignored_best_iou = std::max(
          ignored_best_iou,
          MfPreselectionLogicResult::bboxIou(ignored, candidate));
    }
    if (MfPreselectionLogicResult::isIgnoredTarget(
            candidate, ignored_r2_targets_, params_.grab_verify_iou_threshold)) {
      ++ignored_count;
      if (ignored_detail.empty()) {
        std::ostringstream detail;
        detail << "代表忽略候选=" << detectionSummary(det)
               << " ignored列表数量=" << ignored_r2_targets_.size()
               << " 最高IoU=" << ignored_best_iou
               << " 阈值=" << params_.grab_verify_iou_threshold;
        ignored_detail = detail.str();
      }
      continue;
    }

    CandidateDepthState depth_state;
    const auto bbox_depth = MfPreselectionLogicResult::sampleKfsDepthFromBbox(
        snapshot.depth, det.x1, det.y1, det.x2, det.y2, depth_config,
        params_.kfs_depth_bbox_sample_ratios,
        params_.kfs_depth_bbox_min_success_count);
    if (bbox_depth.has_depth) {
      depth_state.has_depth = true;
      depth_state.depth_m = bbox_depth.depth_m;
      depth_state.source = bbox_depth.source;
      depth_state.detail = bbox_depth.detail;
      ++usable_depth_count;
    } else {
      ++depth_invalid_count;
      const bool replace_depth_detail =
          depth_invalid_detail.empty() ||
          bbox_depth.representative_failure.window_valid_count >
              depth_invalid_best_window_valid_count ||
          (bbox_depth.representative_failure.window_valid_count ==
               depth_invalid_best_window_valid_count &&
           bbox_depth.representative_failure.raw_valid_count >
               depth_invalid_best_raw_valid_count);
      if (replace_depth_detail) {
        depth_invalid_best_window_valid_count =
            bbox_depth.representative_failure.window_valid_count;
        depth_invalid_best_raw_valid_count =
            bbox_depth.representative_failure.raw_valid_count;
        std::ostringstream detail;
        detail << "代表失败候选=" << detectionSummary(det) << " "
               << bbox_depth.detail;
        depth_invalid_detail = detail.str();
      }

      if (allow_depthless_align) {
        const auto mono = MfPreselectionLogicResult::estimateKfsMonocularDepth(
            std::abs(static_cast<double>(det.x2) -
                     static_cast<double>(det.x1)),
            std::abs(static_cast<double>(det.y2) -
                     static_cast<double>(det.y1)),
            kfs_last_real_depth_m_, params_, depth_config.min_depth_m,
            depth_config.max_depth_m);
        if (mono.usable) {
          depth_state.has_depth = true;
          depth_state.depth_m = mono.depth_m;
          depth_state.source =
              MfPreselectionLogicResult::KfsDepthSource::MonocularBbox;
          depth_state.detail = mono.detail;
          ++usable_depth_count;
          ++mono_depth_count;
        } else {
          ++depthless_align_count;
          depth_state.detail = bbox_depth.detail + " " + mono.detail;
        }
      }
    }

    if (!depth_state.has_depth && !allow_depthless_align) {
      continue;
    }

    auto with_depth = candidate;
    if (depth_state.has_depth) {
      with_depth.distance_m = depth_state.depth_m;
    }
    if (depth_state.has_depth && !first_valid_depth.has_value()) {
      first_valid_depth = depth_state.depth_m;
      std::ostringstream detail;
      detail << "首个有效候选=" << detectionSummary(det)
             << " depth=" << depth_state.depth_m
             << " depth_source="
             << MfPreselectionLogicResult::kfsDepthSourceText(
                    depth_state.source)
             << " " << depth_state.detail;
      first_valid_depth_detail = detail.str();
    }
    candidates.push_back(det);
    snapshots.push_back(with_depth);
    depth_states.push_back(depth_state);
  }

  const auto candidateDetail = [&]() {
    std::ostringstream detail;
    detail << detailPrefix(snapshot.display_sequence)
           << " 检测框总数=" << snapshot.detections.size()
           << " 标签匹配数=" << label_match_count
           << " 忽略过滤数=" << ignored_count
           << " 深度失败数=" << depth_invalid_count
           << " 有效深度候选数=" << usable_depth_count
           << " 尺寸估距成功数=" << mono_depth_count
           << " 横移无深度跟踪数=" << depthless_align_count
           << " 参与选择候选数=" << candidates.size();
    if (!ignored_detail.empty()) {
      detail << " " << ignored_detail;
    }
    if (!depth_invalid_detail.empty()) {
      detail << " 失败候选数=" << depth_invalid_count << " "
             << depth_invalid_detail;
    }
    if (!first_valid_depth_detail.empty()) {
      detail << " " << first_valid_depth_detail;
    }
    return detail.str();
  };

  if (label_match_count == 0) {
    std::ostringstream detail;
    detail << candidateDetail()
           << " 检测框摘要=" << detectionsSummary(snapshot.detections);
    recordR2LockReject("no_label_match", detail.str(),
                       snapshot.display_sequence);
    return std::nullopt;
  }
  if (candidates.empty()) {
    const std::string reason =
        (ignored_count > 0 && ignored_count == label_match_count)
            ? "ignored_target"
            : "depth_invalid";
    recordR2LockReject(reason, candidateDetail(), snapshot.display_sequence);
    return std::nullopt;
  }

  std::vector<int> target_class_ids;
  target_class_ids.reserve(candidates.size());
  for (const auto &det : candidates) {
    if (!rc26_vision::isTipTargetClass(det.class_id, target_class_ids)) {
      target_class_ids.push_back(det.class_id);
    }
  }

  const auto selection = rc26_vision::updateTipAlignmentTarget(
      candidates, snapshot.color_bgr.cols, target_class_ids,
      kfs_align_target_lock_state_, makeKfsAlignmentConfig());
  if (!selection.has_target || selection.target.source_index < 0 ||
      static_cast<std::size_t>(selection.target.source_index) >=
          candidates.size()) {
    std::ostringstream detail;
    const int frame_width = snapshot.color_bgr.cols;
    const int target_line_x =
        std::max(0, frame_width / 2) + params_.kfs_align_target_line_offset_px;
    detail << candidateDetail()
           << " 选择器返回目标=" << yesNoText(selection.has_target)
           << " 源索引=" << selection.target.source_index
           << " frame_width=" << frame_width
           << " target_line_offset="
           << params_.kfs_align_target_line_offset_px
           << " target_line_x=" << target_line_x
           << " 候选class集合=" << intVectorSummary(target_class_ids)
           << " 锁定状态=" << yesNoText(kfs_align_target_lock_state_.locked)
           << " lost_count=" << kfs_align_target_lock_state_.lost_count
           << " selection_locked=" << yesNoText(selection.locked)
           << " selection_lost_count=" << selection.lock_lost_count;
    recordR2LockReject("selection_failed", detail.str(),
                       snapshot.display_sequence);
    return std::nullopt;
  }

  const auto selected_index =
      static_cast<std::size_t>(selection.target.source_index);
  const auto &selected_depth = depth_states[selected_index];
  KfsVisualObservation observation;
  observation.target = snapshots[selected_index];
  observation.offset_px = selection.offset_px;
  observation.has_depth = selected_depth.has_depth;
  observation.depth_source = selected_depth.source;
  observation.depth_detail = selected_depth.detail;
  if (selected_depth.has_depth) {
    observation.target.distance_m = selected_depth.depth_m;
  }
  kfs_align_last_observation_ = observation;
  clearR2LockReject();
  return observation;
}

rc26_vision::TipAlignmentConfig
MfPreselectionFlowAction::makeKfsAlignmentConfig() const {
  return MfPreselectionLogicResult::kfsAlignmentConfig(
      params_, kfs_align_yaw_hold_target_);
}

std::optional<rc26_vision::TipHeadingControl>
MfPreselectionFlowAction::kfsVisualAlignHeadingControl() {
  if (!kfs_align_yaw_hold_captured_) {
    if (!odomReady()) {
      if (node_ && !kfs_align_waiting_odom_logged_) {
        RCLCPP_WARN(node_->get_logger(),
                    "梅林预选赛KFS视觉横移等待odom新鲜以捕获tip口径yaw目标：odom_topic=%s",
                    params_.odom_topic.c_str());
        kfs_align_waiting_odom_logged_ = true;
      }
      return std::nullopt;
    }
    kfs_align_yaw_hold_target_ = odom_yaw_;
    kfs_align_yaw_hold_captured_ = true;
    kfs_align_waiting_odom_logged_ = false;
    if (node_) {
      RCLCPP_INFO(node_->get_logger(),
                  "梅林预选赛KFS视觉横移捕获tip口径yaw目标：yaw=%.3f",
                  kfs_align_yaw_hold_target_);
    }
  }

  if (!odomReady()) {
    if (node_ && !kfs_align_waiting_odom_logged_) {
      RCLCPP_WARN(node_->get_logger(),
                  "梅林预选赛KFS视觉横移等待odom新鲜以执行tip口径yaw gate：odom_topic=%s",
                  params_.odom_topic.c_str());
      kfs_align_waiting_odom_logged_ = true;
    }
    return std::nullopt;
  }

  kfs_align_waiting_odom_logged_ = false;
  return rc26_vision::computeTipHeadingControl(odom_yaw_,
                                               makeKfsAlignmentConfig());
}

std::optional<MfPreselectionFlowAction::KfsVisualObservation>
MfPreselectionFlowAction::kfsAlignTimeoutObservation() const {
  if (kfs_align_last_observation_.has_value() &&
      kfs_align_last_observation_->has_depth) {
    return kfs_align_last_observation_;
  }
  return std::nullopt;
}

void MfPreselectionFlowAction::finishKfsAlignFailure(
    const std::string &reason) {
  publishStop();
  if (MfPreselectionLogicResult::mandatoryEntryStair2Retry(
          kfs_pickup_source_, kfs_pickup_entry_high_protocol_)) {
    rememberCurrentKfsPickupForRetry();
    if (scheduleGrabRetryAfterVisibleFailure(reason)) {
      clearKfsVisualPickup();
      return;
    }
  }
  const bool entry_source =
      kfs_pickup_source_ != MfPreselectionPickupSource::None ||
      kfs_pickup_entry_high_protocol_;
  if (!entry_source) {
    if (kfs_pickup_initial_target_.has_value()) {
      ignored_r2_targets_.push_back(*kfs_pickup_initial_target_);
    } else if (kfs_locked_target_.has_value()) {
      ignored_r2_targets_.push_back(*kfs_locked_target_);
    }
  }
  if (node_) {
    const std::string reject_summary = r2LockRejectSummary();
    RCLCPP_WARN(node_->get_logger(),
                "梅林预选赛KFS横移对齐失败但继续原路线：原因=%s 目标=%s 偏移=%dpx 入口目标允许重试=%s%s",
                translateMfFailureReason(reason).c_str(),
                kfs_pickup_initial_target_.has_value()
                    ? kfs_pickup_initial_target_->label.c_str()
                    : "",
                kfs_odom_offset_px_, entry_source ? "是" : "否",
                reject_summary.c_str());
  }
  const Phase failure_phase = kfs_pickup_failure_phase_;
  clearKfsVisualPickup();
  phase_ = failure_phase;
}

void MfPreselectionFlowAction::publishKfsVisualAlignTwist(double vy,
                                                          double wz) {
  publishTwist(0.0, vy, wz);
}

void MfPreselectionFlowAction::beginKfsOdomAxisMotion(
    KfsOdomAxis axis, double distance_m, double max_speed_mps,
    double min_speed_mps, double tolerance_m, double timeout_s,
    std::string label) {
  clearKfsOdomAxisMotion();
  kfs_odom_axis_ = axis;
  kfs_odom_motion_distance_m_ = distance_m;
  kfs_odom_motion_max_speed_mps_ =
      std::max(0.0, std::abs(max_speed_mps));
  kfs_odom_motion_min_speed_mps_ =
      std::min(std::max(0.0, std::abs(min_speed_mps)),
               kfs_odom_motion_max_speed_mps_);
  kfs_odom_motion_tolerance_m_ =
      std::max(kMinSpeed, std::abs(tolerance_m));
  kfs_odom_motion_timeout_s_ = std::max(kMinTimeoutS, timeout_s);
  kfs_odom_motion_label_ = std::move(label);
  kfs_odom_motion_started_ = true;
  kfs_odom_motion_waiting_logged_ = false;
  kfs_odom_motion_stable_ticks_ = 0;
  if (node_) {
    kfs_odom_motion_start_time_ = node_->now();
    RCLCPP_INFO(node_->get_logger(),
                "梅林预选赛KFS odom闭环段启动：%s axis=%s distance=%.3fm min=%.3fm/s max=%.3fm/s tol=%.3fm yaw_tol=%.1fdeg timeout=%.3fs",
                kfs_odom_motion_label_.c_str(),
                axis == KfsOdomAxis::X ? "X" : "Y",
                kfs_odom_motion_distance_m_,
                kfs_odom_motion_min_speed_mps_,
                kfs_odom_motion_max_speed_mps_,
                kfs_odom_motion_tolerance_m_,
                params_.kfs_odom_yaw_tolerance_deg,
                kfs_odom_motion_timeout_s_);
  }
}

MfPreselectionFlowAction::KfsOdomMotionResult
MfPreselectionFlowAction::tickKfsOdomAxisMotion(
    std::string &failure_reason) {
  failure_reason.clear();
  if (!node_ || !kfs_odom_motion_started_) {
    failure_reason = "kfs_odom_motion_not_started";
    publishStop();
    return KfsOdomMotionResult::Failed;
  }
  if ((node_->now() - kfs_odom_motion_start_time_).seconds() >
      kfs_odom_motion_timeout_s_) {
    failure_reason = "kfs_odom_motion_timeout_" + kfs_odom_motion_label_;
    publishStop();
    return KfsOdomMotionResult::Failed;
  }
  if (!odomReady()) {
    kfs_odom_motion_stable_ticks_ = 0;
    publishStop();
    if (!kfs_odom_motion_waiting_logged_) {
      RCLCPP_WARN(node_->get_logger(),
                  "梅林预选赛KFS odom闭环等待odom新鲜：%s，odom_topic=%s",
                  kfs_odom_motion_label_.c_str(), params_.odom_topic.c_str());
      kfs_odom_motion_waiting_logged_ = true;
    }
    return KfsOdomMotionResult::Running;
  }
  if (!kfs_odom_motion_start_captured_) {
    kfs_odom_motion_start_x_ = odom_x_;
    kfs_odom_motion_start_y_ = odom_y_;
    kfs_odom_motion_start_yaw_ = odom_yaw_;
    kfs_odom_motion_start_captured_ = true;
    kfs_odom_motion_waiting_logged_ = false;
    RCLCPP_INFO(node_->get_logger(),
                "梅林预选赛KFS odom闭环已捕获起点：%s start=(%.3f, %.3f, %.3f)",
                kfs_odom_motion_label_.c_str(), kfs_odom_motion_start_x_,
                kfs_odom_motion_start_y_, kfs_odom_motion_start_yaw_);
  }

  const double c = std::cos(kfs_odom_motion_start_yaw_);
  const double s = std::sin(kfs_odom_motion_start_yaw_);
  const double dx = odom_x_ - kfs_odom_motion_start_x_;
  const double dy = odom_y_ - kfs_odom_motion_start_y_;
  const double progress =
      kfs_odom_axis_ == KfsOdomAxis::X ? dx * c + dy * s
                                       : -dx * s + dy * c;
  const double remaining = kfs_odom_motion_distance_m_ - progress;
  const double distance = std::abs(remaining);
  const double yaw_error =
      normalizeAngle(kfs_odom_motion_start_yaw_ - odom_yaw_);
  const double yaw_tolerance_rad =
      std::abs(params_.kfs_odom_yaw_tolerance_deg) * kDeg2Rad;

  if (distance <= kfs_odom_motion_tolerance_m_ &&
      std::abs(yaw_error) <= yaw_tolerance_rad) {
    ++kfs_odom_motion_stable_ticks_;
    publishStop();
    if (kfs_odom_motion_stable_ticks_ >= params_.kfs_odom_stable_ticks) {
      RCLCPP_INFO(node_->get_logger(),
                  "梅林预选赛KFS odom闭环完成：%s progress=%.3fm target=%.3fm remaining=%.3fm yaw_error=%.3frad",
                  kfs_odom_motion_label_.c_str(), progress,
                  kfs_odom_motion_distance_m_, remaining, yaw_error);
      return KfsOdomMotionResult::Succeeded;
    }
    return KfsOdomMotionResult::Running;
  }

  kfs_odom_motion_stable_ticks_ = 0;
  if (kfs_odom_motion_max_speed_mps_ <= 0.0) {
    failure_reason = "kfs_odom_motion_speed_non_positive_" +
                     kfs_odom_motion_label_;
    publishStop();
    return KfsOdomMotionResult::Failed;
  }

  double axis_speed = params_.kfs_odom_xy_kp * remaining;
  if (std::abs(axis_speed) > kfs_odom_motion_max_speed_mps_) {
    axis_speed = std::copysign(kfs_odom_motion_max_speed_mps_, axis_speed);
  }
  const double limited_abs_speed = std::abs(axis_speed);
  if (limited_abs_speed < kfs_odom_motion_min_speed_mps_ &&
      limited_abs_speed > 1e-9) {
    axis_speed = std::copysign(kfs_odom_motion_min_speed_mps_, axis_speed);
  }

  const double wz = headingAngularZ(kfs_odom_motion_start_yaw_);
  if (kfs_odom_axis_ == KfsOdomAxis::X) {
    publishTwist(axis_speed, 0.0, wz);
  } else {
    publishTwist(0.0, axis_speed, wz);
  }
  return KfsOdomMotionResult::Running;
}

void MfPreselectionFlowAction::clearKfsOdomAxisMotion() {
  kfs_odom_axis_ = KfsOdomAxis::X;
  kfs_odom_motion_distance_m_ = 0.0;
  kfs_odom_motion_min_speed_mps_ = 0.0;
  kfs_odom_motion_max_speed_mps_ = 0.0;
  kfs_odom_motion_tolerance_m_ = 0.0;
  kfs_odom_motion_timeout_s_ = 0.0;
  kfs_odom_motion_start_x_ = 0.0;
  kfs_odom_motion_start_y_ = 0.0;
  kfs_odom_motion_start_yaw_ = 0.0;
  kfs_odom_motion_start_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  kfs_odom_motion_started_ = false;
  kfs_odom_motion_start_captured_ = false;
  kfs_odom_motion_waiting_logged_ = false;
  kfs_odom_motion_stable_ticks_ = 0;
  kfs_odom_motion_label_.clear();
}

void MfPreselectionFlowAction::beginKfsVisualPickup(
    bool high_side, MfPreselectionPickupSource source,
    const KfsVisualObservation &observation, Phase success_phase,
    Phase failure_phase, bool direct_exit_on_success,
    bool entry_high_protocol, R2DepthProfile depth_profile) {
  const auto &target = observation.target;
  if (!canPickup()) {
    if (node_) {
      RCLCPP_INFO(node_->get_logger(),
                  "梅林预选赛已达到R2 KFS夹取上限：%d/%d，跳过视觉对齐并继续流程",
                  pickup_count_, params_.max_pickup_count);
    }
    phase_ = success_phase;
    return;
  }
  publishStop();
  kfs_pickup_active_ = true;
  kfs_pickup_high_side_ = high_side;
  kfs_pickup_source_ = source;
  kfs_pickup_success_phase_ = success_phase;
  kfs_pickup_failure_phase_ = failure_phase;
  kfs_pickup_direct_exit_on_success_ = direct_exit_on_success;
  kfs_pickup_entry_high_protocol_ = entry_high_protocol;
  kfs_pickup_depth_profile_ = depth_profile;
  kfs_pickup_origin_phase_ = phase_;
  kfs_pickup_origin_detect_mode_ = detect_mode_;
  kfs_pickup_initial_target_ = target;
  kfs_locked_target_ = target;
  kfs_odom_target_ = target;
  kfs_align_target_lock_state_.reset();
  kfs_align_last_observation_ = observation;
  kfs_align_target_lock_sequence_ = 0;
  kfs_align_lost_count_ = 0;
  kfs_align_stable_count_ = 0;
  kfs_odom_approach_distance_m_ = 0.0;
  kfs_odom_approach_estimated_duration_s_ = 0.0;
  kfs_odom_approach_started_ = false;
  kfs_align_last_sequence_ = target.sequence;
  kfs_align_waiting_verify_frame_ = false;
  kfs_align_yaw_hold_captured_ = false;
  kfs_align_yaw_hold_target_ = 0.0;
  kfs_align_waiting_odom_logged_ = false;
  kfs_align_last_logged_reject_reason_.clear();
  kfs_odom_offset_px_ = observation.offset_px;
  kfs_odom_locked_depth_m_ = target.distance_m;
  kfs_last_real_depth_m_ =
      observation.has_depth &&
              MfPreselectionLogicResult::kfsDepthSourceIsReal(
                  observation.depth_source)
          ? target.distance_m
          : 0.0;
  clearKfsOdomAxisMotion();
  if (node_) {
    phase_start_ = node_->now();
    kfs_align_total_start_ = phase_start_;
    RCLCPP_INFO(node_->get_logger(),
                "梅林预选赛单帧发现KFS，按tip_alignment目标线口径进入视觉横移对齐：source=%s high_side=%s target=%s seq=%ld depth=%s depth_source=%s depth_profile=%s offset=%dpx target_line_offset=%dpx stable_required=%d bbox=[%.1f %.1f %.1f %.1f] success=%s failure=%s direct_exit=%s",
                sourceName(source), high_side ? "是" : "否",
                target.label.c_str(), static_cast<long>(target.sequence),
                metersText(observation.has_depth, target.distance_m).c_str(),
                MfPreselectionLogicResult::kfsDepthSourceText(
                    observation.depth_source),
                depth_profile == R2DepthProfile::Entry ? "entry" : "general",
                observation.offset_px, params_.kfs_align_target_line_offset_px,
                params_.kfs_align_stable_frames, target.x1, target.y1,
                target.x2, target.y2,
                phaseText(success_phase), phaseText(failure_phase),
                direct_exit_on_success ? "是" : "否");
  }
  writeBlackboardState("kfs_visual_align");
  phase_ = Phase::KfsVisualAlign;
}

BT::NodeStatus MfPreselectionFlowAction::tickKfsVisualAlign() {
  if (!node_ || !kfs_pickup_active_) {
    return fail("kfs_visual_align_state_missing");
  }

  if (!kfs_odom_target_.has_value() ||
      !kfs_pickup_initial_target_.has_value()) {
    return fail("kfs_visual_align_target_missing");
  }

  if (tickEntryMcuStopSettle()) {
    return BT::NodeStatus::RUNNING;
  }

  const double total_elapsed =
      (node_->now() - kfs_align_total_start_).seconds();
  if (total_elapsed > params_.kfs_align_timeout_s) {
    const auto timeout_observation = kfsAlignTimeoutObservation();
    if (timeout_observation.has_value() &&
        MfPreselectionLogicResult::kfsAlignTimeoutPickupAllowed(
            timeout_observation->offset_px, timeout_observation->has_depth,
            params_)) {
      const std::string reject_summary = r2LockRejectSummary();
      RCLCPP_WARN(node_->get_logger(),
                  "梅林预选赛KFS横移对齐超时但目标仍在补夹窗口内，继续前向趋近夹取：target=%s offset=%dpx tolerance=%dpx depth=%s depth_source=%s timeout=%.2fs%s",
                  timeout_observation->target.label.c_str(),
                  timeout_observation->offset_px,
                  params_.kfs_align_timeout_pickup_tolerance_px,
                  metersText(timeout_observation->has_depth,
                             timeout_observation->target.distance_m)
                      .c_str(),
                  MfPreselectionLogicResult::kfsDepthSourceText(
                      timeout_observation->depth_source),
                  params_.kfs_align_timeout_s, reject_summary.c_str());
      return beginKfsOdomApproach(*timeout_observation);
    }
    finishKfsAlignFailure("kfs_visual_align_total_timeout");
    return BT::NodeStatus::RUNNING;
  }

  const auto observation = findR2LockObservation(
      kfs_pickup_depth_profile_, R2LockObservationMode::AllowDepthlessForAlign);
  if (!observation.has_value()) {
    const bool has_new_lock_sequence =
        kfs_align_target_lock_sequence_ > 0 &&
        kfs_align_target_lock_sequence_ != kfs_align_last_sequence_;
    if (has_new_lock_sequence) {
      kfs_align_last_sequence_ = kfs_align_target_lock_sequence_;
      ++kfs_align_lost_count_;
      kfs_align_stable_count_ = 0;
    }
    if (has_new_lock_sequence && !last_r2_lock_reject_reason_.empty()) {
      const std::string reject_key = last_r2_lock_reject_reason_;
      if (reject_key != kfs_align_last_logged_reject_reason_) {
        kfs_align_last_logged_reject_reason_ = reject_key;
        const int visible_lost = std::clamp(
            kfs_align_lost_count_, 0, params_.kfs_lost_stop_frames);
        RCLCPP_INFO(node_->get_logger(),
                    "梅林预选赛KFS横移对齐本帧未形成可用R2目标：lost=%d/%d%s",
                    visible_lost, params_.kfs_lost_stop_frames,
                    r2LockRejectSummary().c_str());
      }
    }
    if (kfs_align_lost_count_ >= params_.kfs_lost_stop_frames) {
      publishStop();
      kfs_align_lost_count_ = params_.kfs_lost_stop_frames;
      kfs_align_waiting_verify_frame_ = false;
      return BT::NodeStatus::RUNNING;
    }
    publishStop();
    return BT::NodeStatus::RUNNING;
  }

  if (!kfs_align_waiting_verify_frame_) {
    kfs_align_waiting_verify_frame_ = true;
    RCLCPP_INFO(node_->get_logger(),
                "梅林预选赛KFS使用tip_alignment口径等待稳定视觉帧：target_line=图像中心线+偏置 target_line_offset=%dpx tolerance=%dpx stable=%d lost_stop=%d",
                params_.kfs_align_target_line_offset_px,
                params_.kfs_align_tolerance_px,
                params_.kfs_align_stable_frames,
                params_.kfs_lost_stop_frames);
  }

  const auto heading_control = kfsVisualAlignHeadingControl();
  if (!heading_control.has_value()) {
    publishStop();
    kfs_align_stable_count_ = 0;
    return BT::NodeStatus::RUNNING;
  }

  kfs_align_lost_count_ = 0;
  kfs_odom_offset_px_ = observation->offset_px;
  if (observation->has_depth) {
    kfs_odom_locked_depth_m_ = observation->target.distance_m;
    kfs_odom_target_ = observation->target;
    if (MfPreselectionLogicResult::kfsDepthSourceIsReal(
            observation->depth_source)) {
      kfs_last_real_depth_m_ = observation->target.distance_m;
    }
  }
  kfs_locked_target_ = observation->target;

  const bool new_frame = observation->target.sequence != kfs_align_last_sequence_;
  if (new_frame) {
    kfs_align_last_sequence_ = observation->target.sequence;
  }
  const bool pixel_aligned =
      std::abs(observation->offset_px) <= params_.kfs_align_tolerance_px;
  const bool aligned = pixel_aligned && heading_control->aligned;
  if (aligned) {
    if (!observation->has_depth) {
      if (new_frame) {
        kfs_align_stable_count_ = 0;
      }
      publishStop();
      const std::string wait_key = "aligned_wait_depth";
      if (new_frame && wait_key != kfs_align_last_logged_reject_reason_) {
        kfs_align_last_logged_reject_reason_ = wait_key;
        RCLCPP_INFO(node_->get_logger(),
                    "梅林预选赛KFS tip_alignment已对齐但等待可用深度：label=%s seq=%ld offset=%dpx target_line_offset=%dpx tolerance=%dpx yaw_error=%.3frad depth_source=%s detail={%s}",
                    observation->target.label.c_str(),
                    static_cast<long>(observation->target.sequence),
                    observation->offset_px,
                    params_.kfs_align_target_line_offset_px,
                    params_.kfs_align_tolerance_px,
                    heading_control->yaw_error_rad,
                    MfPreselectionLogicResult::kfsDepthSourceText(
                        observation->depth_source),
                    observation->depth_detail.c_str());
      }
      return BT::NodeStatus::RUNNING;
    }
    if (new_frame) {
      ++kfs_align_stable_count_;
    }
    publishStop();
    if (new_frame) {
      RCLCPP_INFO(node_->get_logger(),
                  "梅林预选赛KFS tip_alignment稳定计数：label=%s seq=%ld offset=%dpx target_line_offset=%dpx tolerance=%dpx yaw_error=%.3frad stable=%d/%d depth=%s depth_source=%s detail={%s}",
                  observation->target.label.c_str(),
                  static_cast<long>(observation->target.sequence),
                  observation->offset_px,
                  params_.kfs_align_target_line_offset_px,
                  params_.kfs_align_tolerance_px,
                  heading_control->yaw_error_rad, kfs_align_stable_count_,
                  params_.kfs_align_stable_frames,
                  metersText(observation->has_depth,
                             observation->target.distance_m)
                      .c_str(),
                  MfPreselectionLogicResult::kfsDepthSourceText(
                      observation->depth_source),
                  observation->depth_detail.c_str());
    }
    if (kfs_align_stable_count_ >= params_.kfs_align_stable_frames) {
      RCLCPP_INFO(node_->get_logger(),
                  "梅林预选赛KFS tip_alignment确认已对齐：offset=%dpx target_line_offset=%dpx tolerance=%dpx yaw_error=%.3frad depth=%s depth_source=%s，进入前向odom闭环趋近规划",
                  observation->offset_px,
                  params_.kfs_align_target_line_offset_px,
                  params_.kfs_align_tolerance_px,
                  heading_control->yaw_error_rad,
                  metersText(observation->has_depth,
                             observation->target.distance_m)
                      .c_str(),
                  MfPreselectionLogicResult::kfsDepthSourceText(
                      observation->depth_source));
      return beginKfsOdomApproach(*observation);
    }
    return BT::NodeStatus::RUNNING;
  }

  if (new_frame) {
    kfs_align_stable_count_ = 0;
  }
  const double vy = heading_control->allow_lateral
                        ? rc26_vision::computeTipAlignmentVy(
                              observation->offset_px, makeKfsAlignmentConfig())
                        : 0.0;
  RCLCPP_INFO(node_->get_logger(),
              "梅林预选赛KFS tip_alignment更新：label=%s seq=%ld offset=%dpx target_line_offset=%dpx tolerance=%dpx pixel_aligned=%s yaw_aligned=%s yaw_gate=%s vy=%.3fm/s wz=%.3frad/s depth=%s depth_source=%s detail={%s}",
              observation->target.label.c_str(),
              static_cast<long>(observation->target.sequence),
              observation->offset_px, params_.kfs_align_target_line_offset_px,
              params_.kfs_align_tolerance_px,
              pixel_aligned ? "是" : "否",
              heading_control->aligned ? "是" : "否",
              heading_control->within_gate ? "是" : "否", vy,
              heading_control->angular_z_radps,
              metersText(observation->has_depth,
                         observation->target.distance_m)
                  .c_str(),
              MfPreselectionLogicResult::kfsDepthSourceText(
                  observation->depth_source),
              observation->depth_detail.c_str());
  publishKfsVisualAlignTwist(vy, heading_control->angular_z_radps);
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus MfPreselectionFlowAction::beginKfsOdomApproach(
    const KfsVisualObservation &observation) {
  if (!node_ || !kfs_pickup_active_) {
    return fail("kfs_odom_approach_state_missing");
  }
  if (!observation.has_depth) {
    finishKfsAlignFailure("kfs_odom_approach_depth_missing");
    return BT::NodeStatus::RUNNING;
  }

  const double planned_distance_m =
      MfPreselectionLogicResult::kfsApproachOdomDistance(
          observation.target.distance_m, params_);
  const double planned_abs_distance_m = std::abs(planned_distance_m);
  const double planned_duration_s =
      MfPreselectionLogicResult::kfsOpenLoopDuration(
          planned_abs_distance_m, params_.kfs_approach_speed_mps);
  if (planned_abs_distance_m > 0.0 && params_.kfs_approach_speed_mps <= 0.0) {
    return fail("kfs_odom_approach_speed_non_positive");
  }
  if (planned_duration_s > params_.kfs_approach_timeout_s) {
    RCLCPP_ERROR(node_->get_logger(),
                 "梅林预选赛KFS odom闭环趋近计划超过安全超时：distance=%.3fm speed=%.3fm/s estimated_duration=%.3fs timeout=%.3fs",
                 planned_abs_distance_m, params_.kfs_approach_speed_mps,
                 planned_duration_s, params_.kfs_approach_timeout_s);
    return fail("kfs_odom_approach_plan_timeout");
  }

  kfs_odom_target_ = observation.target;
  kfs_odom_offset_px_ = observation.offset_px;
  kfs_odom_locked_depth_m_ = observation.target.distance_m;
  if (MfPreselectionLogicResult::kfsDepthSourceIsReal(
          observation.depth_source)) {
    kfs_last_real_depth_m_ = observation.target.distance_m;
  }
  kfs_odom_approach_distance_m_ = planned_distance_m;
  kfs_odom_approach_estimated_duration_s_ = planned_duration_s;
  kfs_odom_approach_started_ = false;
  last_grab_approach_distance_m_ = planned_distance_m;
  clearKfsOdomAxisMotion();

  RCLCPP_INFO(node_->get_logger(),
              "梅林预选赛KFS目标对齐稳定，锁定odom闭环趋近：label=%s offset=%d locked_depth=%.3fm depth_source=%s depth_detail={%s} arm_reach=%.3fm distance=%.3fm speed_max=%.3fm/s speed_min=%.3fm/s estimated_duration=%.3fs",
              observation.target.label.c_str(), observation.offset_px,
              kfs_odom_locked_depth_m_,
              MfPreselectionLogicResult::kfsDepthSourceText(
                  observation.depth_source),
              observation.depth_detail.c_str(), params_.kfs_grab_distance_m,
              kfs_odom_approach_distance_m_, params_.kfs_approach_speed_mps,
              params_.kfs_approach_min_speed_mps,
              kfs_odom_approach_estimated_duration_s_);
  if (!kfs_pickup_high_side_) {
    publishStop();
    writeBlackboardState("kfs_second_arm_lower");
    RCLCPP_INFO(node_->get_logger(),
                "梅林预选赛KFS向下夹取odom闭环趋近前先放下第二节机械臂：cmd=0x%02X feedback=0x%02X",
                static_cast<unsigned int>(clampByte(params_.second_arm_lower_command_id)),
                static_cast<unsigned int>(clampByte(params_.second_arm_lower_done_feedback_id)));
    beginMechanismCommand(clampByte(params_.second_arm_lower_command_id),
                          "ARM_SECOND_LOWER",
                          clampByte(params_.second_arm_lower_done_feedback_id),
                          Phase::KfsSecondArmLower,
                          "kfs_second_arm_lower_failed");
    return BT::NodeStatus::RUNNING;
  }
  startKfsOdomApproach();
  return BT::NodeStatus::RUNNING;
}

void MfPreselectionFlowAction::startKfsOdomApproach() {
  if (!node_ || !kfs_pickup_active_ || !kfs_odom_target_.has_value()) {
    return;
  }
  kfs_odom_approach_started_ = true;
  writeBlackboardState("kfs_odom_approach");
  phase_ = Phase::KfsOdomApproach;
  if (std::abs(kfs_odom_approach_distance_m_) <=
      params_.kfs_approach_odom_tolerance_m) {
    publishStop();
    RCLCPP_INFO(node_->get_logger(),
                "梅林预选赛KFS锁定深度已在机械臂可触达范围内，直接发送夹取命令");
    beginGrab(kfs_pickup_high_side_, kfs_pickup_source_, *kfs_odom_target_,
              kfs_pickup_success_phase_, kfs_pickup_failure_phase_,
              kfs_pickup_direct_exit_on_success_,
              kfs_pickup_entry_high_protocol_);
    clearKfsVisualPickup();
  } else {
    beginKfsOdomAxisMotion(KfsOdomAxis::X, kfs_odom_approach_distance_m_,
                           params_.kfs_approach_speed_mps,
                           params_.kfs_approach_min_speed_mps,
                           params_.kfs_approach_odom_tolerance_m,
                           params_.kfs_approach_timeout_s,
                           "kfs_forward_approach");
  }
}

BT::NodeStatus MfPreselectionFlowAction::tickKfsOdomApproach() {
  if (!node_ || !kfs_pickup_active_ || !kfs_odom_target_.has_value()) {
    return fail("kfs_odom_approach_target_missing");
  }
  if (!kfs_odom_approach_started_) {
    startKfsOdomApproach();
    return BT::NodeStatus::RUNNING;
  }

  std::string failure_reason;
  const auto motion_result = tickKfsOdomAxisMotion(failure_reason);
  if (motion_result == KfsOdomMotionResult::Succeeded) {
    publishStop();
    RCLCPP_INFO(node_->get_logger(),
                "梅林预选赛KFS odom闭环趋近完成：locked_depth=%.3fm planned_distance=%.3fm，发送夹取",
                kfs_odom_locked_depth_m_, kfs_odom_approach_distance_m_);
    beginGrab(kfs_pickup_high_side_, kfs_pickup_source_, *kfs_odom_target_,
              kfs_pickup_success_phase_, kfs_pickup_failure_phase_,
              kfs_pickup_direct_exit_on_success_,
              kfs_pickup_entry_high_protocol_);
    clearKfsVisualPickup();
    return BT::NodeStatus::RUNNING;
  }
  if (motion_result == KfsOdomMotionResult::Failed) {
    publishStop();
    return fail(failure_reason.empty() ? "kfs_odom_approach_runtime_failed"
                                       : failure_reason);
  }

  return BT::NodeStatus::RUNNING;
}

void MfPreselectionFlowAction::clearKfsVisualPickup() {
  kfs_pickup_active_ = false;
  kfs_pickup_high_side_ = true;
  kfs_pickup_source_ = MfPreselectionPickupSource::None;
  kfs_pickup_success_phase_ = Phase::Done;
  kfs_pickup_failure_phase_ = Phase::Done;
  kfs_pickup_direct_exit_on_success_ = false;
  kfs_pickup_entry_high_protocol_ = false;
  kfs_pickup_depth_profile_ = R2DepthProfile::General;
  kfs_pickup_origin_phase_ = Phase::Done;
  kfs_pickup_origin_detect_mode_ = DetectMode::Entry2;
  kfs_pickup_initial_target_.reset();
  kfs_locked_target_.reset();
  kfs_odom_target_.reset();
  kfs_align_target_lock_state_.reset();
  kfs_align_last_observation_.reset();
  kfs_align_target_lock_sequence_ = 0;
  kfs_align_stable_count_ = 0;
  kfs_align_lost_count_ = 0;
  kfs_align_last_sequence_ = 0;
  kfs_align_total_start_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  kfs_entry_mcu_stop_settle_active_ = false;
  kfs_entry_mcu_stop_settle_until_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  kfs_entry_mcu_stop_settle_duration_s_ = 0.0;
  kfs_entry_mcu_stop_settle_speed_mps_ = 0.0;
  kfs_entry_mcu_stop_settle_done_logged_ = false;
  kfs_align_waiting_verify_frame_ = false;
  kfs_align_yaw_hold_captured_ = false;
  kfs_align_yaw_hold_target_ = 0.0;
  kfs_align_waiting_odom_logged_ = false;
  kfs_align_last_logged_reject_reason_.clear();
  kfs_odom_offset_px_ = 0;
  kfs_odom_locked_depth_m_ = 0.0;
  kfs_last_real_depth_m_ = 0.0;
  kfs_odom_approach_distance_m_ = 0.0;
  kfs_odom_approach_estimated_duration_s_ = 0.0;
  kfs_odom_approach_started_ = false;
  clearKfsOdomAxisMotion();
}

void MfPreselectionFlowAction::beginGrab(
    bool high_side, MfPreselectionPickupSource source,
    const MfPreselectionTargetSnapshot &target, Phase success_phase,
    Phase failure_phase, bool direct_exit_on_success,
    bool entry_high_protocol) {
  // 夹取命令 ACK 只表示 transport 收到通用确认；真正计数要等后续
  // GrabVerify 阶段确认夹取前的同一视觉目标已经连续新帧消失。
  if (!canPickup()) {
    if (node_) {
      RCLCPP_INFO(node_->get_logger(),
                  "梅林预选赛已达到R2 KFS夹取上限：%d/%d，跳过夹取继续流程",
                  pickup_count_, params_.max_pickup_count);
    }
    phase_ = success_phase;
    return;
  }
  grab_success_phase_ = success_phase;
  grab_failure_phase_ = failure_phase;
  grab_success_direct_exit_ = direct_exit_on_success;
  post_grab_center_next_phase_ = Phase::Done;
  last_grab_retry_context_valid_ = true;
  last_grab_high_side_ = high_side;
  last_grab_source_ = source;
  last_grab_success_phase_ = success_phase;
  last_grab_failure_phase_ = failure_phase;
  last_grab_direct_exit_on_success_ = direct_exit_on_success;
  last_grab_entry_high_protocol_ = entry_high_protocol;
  last_grab_depth_profile_ = kfs_pickup_depth_profile_;
  last_grab_target_ = target;
  last_grab_origin_phase_ = kfs_pickup_origin_phase_;
  last_grab_origin_detect_mode_ = kfs_pickup_origin_detect_mode_;
  // pending_grab_commit_ 让计数更新延迟到物理夹取视觉验证成功之后，避免
  // ACK 成功但空夹时仍消耗本局最多夹取次数。
  pending_grab_commit_ = true;
  pending_grab_source_ = source;
  pending_grab_entry_high_protocol_ = entry_high_protocol;
  pending_grab_target_ = target;
  const uint8_t command_id =
      MfPreselectionLogicResult::grabCommandForPickup(high_side, source,
                                                      entry_high_protocol,
                                                      params_);
  const int done_feedback_id =
      MfPreselectionLogicResult::grabDoneFeedbackForPickup(high_side, source,
                                                           entry_high_protocol,
                                                           params_);
  const char *command_label =
      done_feedback_id >= 0
          ? "ENTRY_GRAB_KFS_UP"
          : (high_side ? "GRAB_KFS_UP" : "GRAB_KFS_DOWN");
  if (node_) {
    if (done_feedback_id >= 0) {
      RCLCPP_INFO(
          node_->get_logger(),
          "梅林预选赛开始入口高夹取R2 KFS：command=%s(0x%02X)，等待完成反馈=0x%02X，来源=%s，目标=%s seq=%ld bbox=[%.1f %.1f %.1f %.1f]，当前计数=%d/%d",
          command_label, static_cast<unsigned int>(command_id),
          static_cast<unsigned int>(done_feedback_id), sourceName(source),
          target.label.c_str(), static_cast<long>(target.sequence), target.x1,
          target.y1, target.x2, target.y2, pickup_count_,
          params_.max_pickup_count);
    } else {
      RCLCPP_INFO(
          node_->get_logger(),
          "梅林预选赛开始夹取R2 KFS：command=%s(0x%02X)，高侧=%s，来源=%s，目标=%s seq=%ld bbox=[%.1f %.1f %.1f %.1f]，当前计数=%d/%d",
          command_label, static_cast<unsigned int>(command_id),
          high_side ? "是" : "否", sourceName(source), target.label.c_str(),
          static_cast<long>(target.sequence), target.x1, target.y1, target.x2,
          target.y2, pickup_count_, params_.max_pickup_count);
    }
  }
  beginMechanismCommand(command_id, command_label, done_feedback_id,
                        Phase::GrabVerify, "grab_kfs_failed");
}

void MfPreselectionFlowAction::beginGrabVerify() {
  if (!node_ || !pending_grab_target_.has_value()) {
    finishGrabVerificationFailure("grab_verify_target_missing");
    return;
  }
  phase_start_ = node_->now();
  grab_verify_lost_count_ = 0;
  grab_verify_last_sequence_ = pending_grab_target_->sequence;
  grab_verify_seen_new_frame_ = false;
  grab_verify_visible_logged_ = false;
  grab_verify_last_logged_lost_count_ = 0;
  writeBlackboardState("grab_verify");
  RCLCPP_INFO(node_->get_logger(),
              "梅林预选赛开始物理夹取视觉验证：target=%s seq=%ld timeout=%.2fs lost=%d iou>=%.2f",
              pending_grab_target_->label.c_str(),
              static_cast<long>(pending_grab_target_->sequence),
              params_.grab_verify_timeout_s,
              params_.grab_verify_lost_stable_frames,
              params_.grab_verify_iou_threshold);
  phase_ = Phase::GrabVerify;
}

BT::NodeStatus MfPreselectionFlowAction::tickGrabVerify() {
  publishStop();
  if (!node_ || !pending_grab_target_.has_value()) {
    finishGrabVerificationFailure("grab_verify_target_missing");
    return BT::NodeStatus::RUNNING;
  }

  const double elapsed = (node_->now() - phase_start_).seconds();
  rc26_vision::VisionInferenceManager::FrameSnapshot snapshot;
  const bool has_snapshot = vision_ && vision_->isRunning() &&
                            vision_->getLatestFrameSnapshot(snapshot) &&
                            snapshot.has_display &&
                            snapshot.display_sequence > 0;
  if (!has_snapshot || snapshot.display_sequence <= grab_verify_last_sequence_) {
    if (elapsed >= params_.grab_verify_timeout_s) {
      finishGrabVerificationFailure("grab_verify_no_new_frame");
    }
    return BT::NodeStatus::RUNNING;
  }

  grab_verify_seen_new_frame_ = true;
  grab_verify_last_sequence_ = snapshot.display_sequence;
  bool still_visible = false;
  double best_iou = 0.0;
  for (const auto &det : snapshot.detections) {
    const MfPreselectionTargetSnapshot candidate =
        rc26_vision::makeVisualTargetSnapshot(det, snapshot.display_sequence);
    if (candidate.label != pending_grab_target_->label) {
      continue;
    }
    const double iou =
        MfPreselectionLogicResult::bboxIou(*pending_grab_target_, candidate);
    best_iou = std::max(best_iou, iou);
    if (iou >= params_.grab_verify_iou_threshold) {
      still_visible = true;
      break;
    }
  }

  if (still_visible) {
    grab_verify_lost_count_ = 0;
    grab_verify_last_logged_lost_count_ = 0;
    if (!grab_verify_visible_logged_) {
      RCLCPP_INFO(node_->get_logger(),
                  "梅林预选赛物理夹取验证：原目标仍可见 target=%s seq=%ld iou=%.3f",
                  pending_grab_target_->label.c_str(),
                  static_cast<long>(snapshot.display_sequence), best_iou);
      grab_verify_visible_logged_ = true;
    }
  } else {
    ++grab_verify_lost_count_;
    grab_verify_visible_logged_ = false;
    if (grab_verify_lost_count_ != grab_verify_last_logged_lost_count_) {
      RCLCPP_INFO(node_->get_logger(),
                  "梅林预选赛物理夹取验证：原目标未匹配 stable=%d/%d seq=%ld best_iou=%.3f",
                  grab_verify_lost_count_,
                  params_.grab_verify_lost_stable_frames,
                  static_cast<long>(snapshot.display_sequence), best_iou);
      grab_verify_last_logged_lost_count_ = grab_verify_lost_count_;
    }
    if (grab_verify_lost_count_ >= params_.grab_verify_lost_stable_frames) {
      const bool center_align_after_grab =
          MfPreselectionLogicResult::postGrabCenterAlignRequired(
              pending_grab_source_, pending_grab_entry_high_protocol_);
      Phase settle_next_phase = grab_success_phase_;
      if (center_align_after_grab) {
        post_grab_center_next_phase_ = grab_success_phase_;
        settle_next_phase = Phase::PostGrabCenterAlign;
      } else {
        post_grab_center_next_phase_ = Phase::Done;
      }
      if (grab_success_direct_exit_) {
        direct_exit_mode_ = true;
      }
      commitPendingGrab();
      RCLCPP_INFO(node_->get_logger(),
                  "梅林预选赛物理夹取确认成功：连续%d帧未识别到原目标，进入夹取稳定等待，随后%s",
                  params_.grab_verify_lost_stable_frames,
                  center_align_after_grab ? "先归当前格中心" : "继续原流程");
      beginZeroHold(params_.grab_settle_s, settle_next_phase, "grab_settle");
      return BT::NodeStatus::RUNNING;
    }
  }

  if (elapsed >= params_.grab_verify_timeout_s) {
    const char *failure_reason =
        !grab_verify_seen_new_frame_
            ? "grab_verify_no_new_frame"
            : (still_visible ? "grab_verify_target_still_visible"
                             : "grab_verify_target_not_stably_lost");
    finishGrabVerificationFailure(failure_reason);
  }
  return BT::NodeStatus::RUNNING;
}

void MfPreselectionFlowAction::commitPendingGrab() {
  if (!pending_grab_commit_) {
    return;
  }
  ++pickup_count_;
  entry_pickup_done_ =
      entry_pickup_done_ || pending_grab_entry_high_protocol_ ||
      pending_grab_source_ != MfPreselectionPickupSource::None;
  rememberPickupSource(pending_grab_source_);
  if (config().blackboard) {
    config().blackboard->set("mf_preselect_pickup_count", pickup_count_);
  }
  if (node_) {
    RCLCPP_INFO(node_->get_logger(),
                "梅林预选赛R2 KFS夹取计数已更新：%d/%d，来源=%s",
                pickup_count_, params_.max_pickup_count,
                sourceName(pickup_source_));
  }
  pending_grab_commit_ = false;
  pending_grab_source_ = MfPreselectionPickupSource::None;
  pending_grab_entry_high_protocol_ = false;
  pending_grab_target_.reset();
  grab_success_direct_exit_ = false;
}

void MfPreselectionFlowAction::finishGrabVerificationFailure(
    const std::string &reason) {
  if (scheduleGrabRetryAfterVisibleFailure(reason)) {
    pending_grab_commit_ = false;
    pending_grab_source_ = MfPreselectionPickupSource::None;
    pending_grab_entry_high_protocol_ = false;
    pending_grab_target_.reset();
    grab_success_direct_exit_ = false;
    post_grab_center_next_phase_ = Phase::Done;
    grab_verify_lost_count_ = 0;
    grab_verify_last_sequence_ = 0;
    grab_verify_seen_new_frame_ = false;
    grab_verify_visible_logged_ = false;
    grab_verify_last_logged_lost_count_ = 0;
    return;
  }
  if (pending_grab_target_.has_value()) {
    ignored_r2_targets_.push_back(*pending_grab_target_);
  }
  if (node_) {
    const char *label =
        pending_grab_target_.has_value() ? pending_grab_target_->label.c_str() : "";
    RCLCPP_WARN(node_->get_logger(),
                "梅林预选赛物理夹取验证失败但继续流程：原因=%s 目标=%s，忽略该目标且不更新计数",
                translateMfFailureReason(reason).c_str(), label);
  }
  pending_grab_commit_ = false;
  pending_grab_source_ = MfPreselectionPickupSource::None;
  pending_grab_entry_high_protocol_ = false;
  pending_grab_target_.reset();
  grab_success_direct_exit_ = false;
  post_grab_center_next_phase_ = Phase::Done;
  grab_verify_lost_count_ = 0;
  grab_verify_last_sequence_ = 0;
  grab_verify_seen_new_frame_ = false;
  grab_verify_visible_logged_ = false;
  grab_verify_last_logged_lost_count_ = 0;
  phase_ = grab_failure_phase_;
}

uint8_t MfPreselectionFlowAction::clampByte(int value) const {
  return static_cast<uint8_t>(std::clamp(value, 0, 255));
}

rclcpp::Duration MfPreselectionFlowAction::seconds(double value) const {
  return rclcpp::Duration::from_seconds(value);
}

void loadMfPreselectionParams(rclcpp::Node &node,
                              const BT::Blackboard::Ptr &blackboard) {
  // 参数集中由 decision_node 启动时声明并写入黑板；MfPreselectionFlowAction
  // 运行中不监听参数变化。这里按语义分组声明，便于和 r2_runtime.yaml 对照。
  MfPreselectionParams p;
  int mirror_sign = 1;
  if (blackboard) {
    (void)blackboard->get("team_mirror_sign", mirror_sign);
  }
  mirror_sign = normalizedMirrorSign(mirror_sign);
  p.field_mirror_sign = mirror_sign;

  // 视觉模型、标签和深度有效区间。R2/R1/假 KFS 的语义完全由这些标签配置决定。
  p.vision_config_file =
      node.declare_parameter<std::string>("mf_preselect_vision_config_file",
                                          p.vision_config_file);
  p.model_id =
      node.declare_parameter<std::string>("mf_preselect_model_id", p.model_id);
  p.r2_target_label_prefixes =
      node.declare_parameter<std::vector<std::string>>(
          "mf_preselect_r2_target_label_prefixes", p.r2_target_label_prefixes);
  p.r2_target_labels = node.declare_parameter<std::vector<std::string>>(
      "mf_preselect_r2_target_labels", p.r2_target_labels);
  p.r1_blocking_labels = node.declare_parameter<std::vector<std::string>>(
      "mf_preselect_r1_blocking_labels", p.r1_blocking_labels);
  p.r1_blocking_label_prefixes =
      node.declare_parameter<std::vector<std::string>>(
          "mf_preselect_r1_blocking_label_prefixes",
          p.r1_blocking_label_prefixes);
  p.r1_kfs_min_score = node.declare_parameter<double>(
      "mf_preselect_r1_kfs_min_score", p.r1_kfs_min_score);
  p.fake_label_prefixes = node.declare_parameter<std::vector<std::string>>(
      "mf_preselect_fake_label_prefixes", p.fake_label_prefixes);
  p.fake_labels = node.declare_parameter<std::vector<std::string>>(
      "mf_preselect_fake_labels", p.fake_labels);
  p.depth_min_m =
      node.declare_parameter<double>("mf_preselect_depth_min_m", p.depth_min_m);
  p.depth_max_m =
      node.declare_parameter<double>("mf_preselect_depth_max_m", p.depth_max_m);
  p.entry_depth_min_m = node.declare_parameter<double>(
      "mf_preselect_entry_depth_min_m", p.entry_depth_min_m);
  p.entry_depth_max_m = node.declare_parameter<double>(
      "mf_preselect_entry_depth_max_m", p.entry_depth_max_m);
  p.detect_seen_stable_frames = node.declare_parameter<int>(
      "mf_preselect_detect_seen_stable_frames", p.detect_seen_stable_frames);
  p.detect_lost_stable_frames = node.declare_parameter<int>(
      "mf_preselect_detect_lost_stable_frames", p.detect_lost_stable_frames);
  p.entry_detect_timeout_s = node.declare_parameter<double>(
      "mf_preselect_entry_detect_timeout_s", p.entry_detect_timeout_s);
  p.scan_detect_timeout_s = node.declare_parameter<double>(
      "mf_preselect_scan_detect_timeout_s", p.scan_detect_timeout_s);
  p.entry_interrupt_max_offset_px = node.declare_parameter<int>(
      "mf_preselect_entry_interrupt_max_offset_px",
      p.entry_interrupt_max_offset_px);
  p.entry_interrupt_dynamic_comp_enable = node.declare_parameter<bool>(
      "mf_preselect_entry_interrupt_dynamic_comp_enable",
      p.entry_interrupt_dynamic_comp_enable);
  p.entry_interrupt_latency_s = node.declare_parameter<double>(
      "mf_preselect_entry_interrupt_latency_s",
      p.entry_interrupt_latency_s);
  p.entry_interrupt_fx_px = node.declare_parameter<double>(
      "mf_preselect_entry_interrupt_fx_px", p.entry_interrupt_fx_px);
  p.entry_interrupt_extra_px_min = node.declare_parameter<int>(
      "mf_preselect_entry_interrupt_extra_px_min",
      p.entry_interrupt_extra_px_min);
  p.entry_interrupt_extra_px_max = node.declare_parameter<int>(
      "mf_preselect_entry_interrupt_extra_px_max",
      p.entry_interrupt_extra_px_max);
  p.entry_mcu_stop_settle_enable = node.declare_parameter<bool>(
      "mf_preselect_entry_mcu_stop_settle_enable",
      p.entry_mcu_stop_settle_enable);
  p.entry_mcu_vy_acc_mps2 = node.declare_parameter<double>(
      "mf_preselect_entry_mcu_vy_acc_mps2", p.entry_mcu_vy_acc_mps2);
  p.entry_mcu_stop_margin_s = node.declare_parameter<double>(
      "mf_preselect_entry_mcu_stop_margin_s",
      p.entry_mcu_stop_margin_s);
  p.entry_mcu_stop_max_wait_s = node.declare_parameter<double>(
      "mf_preselect_entry_mcu_stop_max_wait_s",
      p.entry_mcu_stop_max_wait_s);

  // R2 KFS 夹取前视觉横移和前向 odom 趋近参数。入口区使用独立
  // entry_depth_min/max 窗口，其余检测使用 depth_min/max；横移阶段用视觉
  // offset 闭环发布 vy，趋近阶段只闭环执行锁定深度规划出的距离。
  p.kfs_align_tolerance_px = node.declare_parameter<int>(
      "mf_preselect_kfs_align_tolerance_px", p.kfs_align_tolerance_px);
  p.kfs_align_target_line_offset_px = node.declare_parameter<int>(
      "mf_preselect_kfs_align_target_line_offset_px",
      p.kfs_align_target_line_offset_px);
  p.kfs_align_stable_frames = node.declare_parameter<int>(
      "mf_preselect_kfs_align_stable_frames", p.kfs_align_stable_frames);
  p.kfs_align_max_jump_px = node.declare_parameter<int>(
      "mf_preselect_kfs_align_max_jump_px", p.kfs_align_max_jump_px);
  p.kfs_align_kp = node.declare_parameter<double>(
      "mf_preselect_kfs_align_kp", p.kfs_align_kp);
  p.kfs_align_min_speed_mps = node.declare_parameter<double>(
      "mf_preselect_kfs_align_min_speed_mps", p.kfs_align_min_speed_mps);
  p.kfs_align_max_speed_mps = node.declare_parameter<double>(
      "mf_preselect_kfs_align_max_speed_mps", p.kfs_align_max_speed_mps);
  p.kfs_align_timeout_s = node.declare_parameter<double>(
      "mf_preselect_kfs_align_timeout_s", p.kfs_align_timeout_s);
  p.kfs_align_timeout_pickup_tolerance_px = node.declare_parameter<int>(
      "mf_preselect_kfs_align_timeout_pickup_tolerance_px",
      p.kfs_align_timeout_pickup_tolerance_px);
  p.kfs_align_heading_gate_deg = node.declare_parameter<double>(
      "mf_preselect_kfs_align_heading_gate_deg",
      p.kfs_align_heading_gate_deg);
  p.kfs_lost_stop_frames = node.declare_parameter<int>(
      "mf_preselect_kfs_lost_stop_frames", p.kfs_lost_stop_frames);
  p.kfs_invert_lateral_direction = node.declare_parameter<bool>(
      "mf_preselect_kfs_invert_lateral_direction",
      p.kfs_invert_lateral_direction);
  p.kfs_odom_xy_kp = node.declare_parameter<double>(
      "mf_preselect_kfs_odom_xy_kp", p.kfs_odom_xy_kp);
  p.kfs_approach_odom_tolerance_m = node.declare_parameter<double>(
      "mf_preselect_kfs_approach_odom_tolerance_m",
      p.kfs_approach_odom_tolerance_m);
  p.kfs_odom_yaw_tolerance_deg = node.declare_parameter<double>(
      "mf_preselect_kfs_odom_yaw_tolerance_deg",
      p.kfs_odom_yaw_tolerance_deg);
  p.kfs_odom_stable_ticks = node.declare_parameter<int>(
      "mf_preselect_kfs_odom_stable_ticks", p.kfs_odom_stable_ticks);
  p.kfs_approach_speed_mps = node.declare_parameter<double>(
      "mf_preselect_kfs_approach_speed_mps", p.kfs_approach_speed_mps);
  p.kfs_approach_min_speed_mps = node.declare_parameter<double>(
      "mf_preselect_kfs_approach_min_speed_mps",
      p.kfs_approach_min_speed_mps);
  p.kfs_approach_x_sign = node.declare_parameter<int>(
      "mf_preselect_kfs_approach_x_sign", p.kfs_approach_x_sign);
  p.kfs_approach_timeout_s = node.declare_parameter<double>(
      "mf_preselect_kfs_approach_timeout_s", p.kfs_approach_timeout_s);
  p.kfs_grab_distance_m = node.declare_parameter<double>(
      "mf_preselect_kfs_grab_distance_m", p.kfs_grab_distance_m);
  p.kfs_mono_distance_fallback_enable = node.declare_parameter<bool>(
      "mf_preselect_kfs_mono_distance_fallback_enable",
      p.kfs_mono_distance_fallback_enable);
  p.kfs_mono_target_width_m = node.declare_parameter<double>(
      "mf_preselect_kfs_mono_target_width_m",
      p.kfs_mono_target_width_m);
  p.kfs_mono_target_height_m = node.declare_parameter<double>(
      "mf_preselect_kfs_mono_target_height_m",
      p.kfs_mono_target_height_m);
  p.kfs_mono_fx_px = node.declare_parameter<double>(
      "mf_preselect_kfs_mono_fx_px", p.kfs_mono_fx_px);
  p.kfs_mono_fy_px = node.declare_parameter<double>(
      "mf_preselect_kfs_mono_fy_px", p.kfs_mono_fy_px);
  p.kfs_mono_min_bbox_px = node.declare_parameter<int>(
      "mf_preselect_kfs_mono_min_bbox_px", p.kfs_mono_min_bbox_px);
  p.kfs_mono_max_delta_from_locked_m = node.declare_parameter<double>(
      "mf_preselect_kfs_mono_max_delta_from_locked_m",
      p.kfs_mono_max_delta_from_locked_m);
  p.kfs_depth_roi_size = node.declare_parameter<int>(
      "mf_preselect_kfs_depth_roi_size", p.kfs_depth_roi_size);
  p.kfs_depth_min_valid_count = node.declare_parameter<int>(
      "mf_preselect_kfs_depth_min_valid_count",
      p.kfs_depth_min_valid_count);
  p.kfs_depth_bbox_sample_ratios =
      node.declare_parameter<std::vector<double>>(
          "mf_preselect_kfs_depth_bbox_sample_ratios",
          p.kfs_depth_bbox_sample_ratios);
  p.kfs_depth_bbox_min_success_count = node.declare_parameter<int>(
      "mf_preselect_kfs_depth_bbox_min_success_count",
      p.kfs_depth_bbox_min_success_count);

  // 夹取次数和夹爪命令。max_pickup_count=0 可用于路线/避障干跑。
  p.max_pickup_count = node.declare_parameter<int>(
      "mf_preselect_max_pickup_count", p.max_pickup_count);
  p.grab_settle_s = node.declare_parameter<double>(
      "mf_preselect_grab_settle_s", p.grab_settle_s);
  p.grab_verify_timeout_s = node.declare_parameter<double>(
      "mf_preselect_grab_verify_timeout_s", p.grab_verify_timeout_s);
  p.grab_verify_lost_stable_frames = node.declare_parameter<int>(
      "mf_preselect_grab_verify_lost_stable_frames",
      p.grab_verify_lost_stable_frames);
  p.grab_verify_iou_threshold = node.declare_parameter<double>(
      "mf_preselect_grab_verify_iou_threshold",
      p.grab_verify_iou_threshold);
  p.grab_kfs_up_command_id = node.declare_parameter<int>(
      "mf_preselect_grab_kfs_up_command_id", p.grab_kfs_up_command_id);
  p.grab_kfs_down_command_id = node.declare_parameter<int>(
      "mf_preselect_grab_kfs_down_command_id", p.grab_kfs_down_command_id);
  p.entry_grab_kfs_up_command_id = node.declare_parameter<int>(
      "mf_preselect_entry_grab_kfs_up_command_id",
      p.entry_grab_kfs_up_command_id);
  p.entry_grab_kfs_up_done_feedback_id = node.declare_parameter<int>(
      "mf_preselect_entry_grab_kfs_up_done_feedback_id",
      p.entry_grab_kfs_up_done_feedback_id);

  // 运动、机构 service 和反馈 topic。cmd_vel 是本状态机直接发布的运动权威。
  p.cmd_vel_topic = node.declare_parameter<std::string>(
      "mf_preselect_cmd_vel_topic", p.cmd_vel_topic);
  p.odom_topic = node.declare_parameter<std::string>(
      "mf_preselect_odom_topic", p.odom_topic);
  p.send_command_service = node.declare_parameter<std::string>(
      "mf_preselect_send_command_service", p.send_command_service);
  p.feedback_topic = node.declare_parameter<std::string>(
      "mf_preselect_feedback_topic", p.feedback_topic);
  p.command_timeout_s = node.declare_parameter<double>(
      "mf_preselect_command_timeout_s", p.command_timeout_s);
  p.arm_high_raise_command_id = node.declare_parameter<int>(
      "mf_preselect_arm_high_raise_command_id", p.arm_high_raise_command_id);
  p.arm_high_raise_done_feedback_id = node.declare_parameter<int>(
      "mf_preselect_arm_high_raise_done_feedback_id",
      p.arm_high_raise_done_feedback_id);
  p.arm_raise_command_id = node.declare_parameter<int>(
      "mf_preselect_arm_raise_command_id", p.arm_raise_command_id);
  p.arm_lower_command_id = node.declare_parameter<int>(
      "mf_preselect_arm_lower_command_id", p.arm_lower_command_id);
  p.arm_raise_done_feedback_id = node.declare_parameter<int>(
      "mf_preselect_arm_raise_done_feedback_id", p.arm_raise_done_feedback_id);
  p.arm_lower_done_feedback_id = node.declare_parameter<int>(
      "mf_preselect_arm_lower_done_feedback_id", p.arm_lower_done_feedback_id);
  p.second_arm_lower_command_id = node.declare_parameter<int>(
      "mf_preselect_second_arm_lower_command_id",
      p.second_arm_lower_command_id);
  p.second_arm_lower_done_feedback_id = node.declare_parameter<int>(
      "mf_preselect_second_arm_lower_done_feedback_id",
      p.second_arm_lower_done_feedback_id);

  // 入口横移、相对移动和直出兜底参数。距离按绝对值规范化，方向由调用方速度符号表达。
  p.entry_probe_left_distance_m = node.declare_parameter<double>(
      "mf_preselect_entry_probe_left_distance_m",
      p.entry_probe_left_distance_m);
  p.entry_probe_right_sweep_distance_m = node.declare_parameter<double>(
      "mf_preselect_entry_probe_right_sweep_distance_m",
      p.entry_probe_right_sweep_distance_m);
  p.entry_probe_return_distance_m = node.declare_parameter<double>(
      "mf_preselect_entry_probe_return_distance_m",
      p.entry_probe_return_distance_m);
  p.lateral_probe_speed_mps = node.declare_parameter<double>(
      "mf_preselect_lateral_probe_speed_mps", p.lateral_probe_speed_mps);
  p.move_tolerance_m = node.declare_parameter<double>(
      "mf_preselect_move_tolerance_m", p.move_tolerance_m);
  p.move_timeout_s = node.declare_parameter<double>(
      "mf_preselect_move_timeout_s", p.move_timeout_s);
  p.direct_exit_drive_distance_m = node.declare_parameter<double>(
      "mf_preselect_direct_exit_drive_distance_m",
      p.direct_exit_drive_distance_m);
  p.direct_exit_drive_speed_mps = node.declare_parameter<double>(
      "mf_preselect_direct_exit_drive_speed_mps", p.direct_exit_drive_speed_mps);

  // 转向和 heading hold 参数。*_yaw_rad 是比赛路线语义，P 控参数是执行层闭环。
  p.exit_yaw_rad =
      node.declare_parameter<double>("mf_preselect_exit_yaw_rad", p.exit_yaw_rad);
  p.stair1_direction_yaw_rad = node.declare_parameter<double>(
      "mf_preselect_stair1_direction_yaw_rad", p.stair1_direction_yaw_rad);
  p.stair3_direction_yaw_rad = node.declare_parameter<double>(
      "mf_preselect_stair3_direction_yaw_rad", p.stair3_direction_yaw_rad);
  p.row_scan_left_yaw_delta_rad = node.declare_parameter<double>(
      "mf_preselect_row_scan_left_yaw_delta_rad",
      p.row_scan_left_yaw_delta_rad);
  p.row_scan_back_yaw_delta_rad = node.declare_parameter<double>(
      "mf_preselect_row_scan_back_yaw_delta_rad",
      p.row_scan_back_yaw_delta_rad);
  p.row4_exit_turn_yaw_rad = node.declare_parameter<double>(
      "mf_preselect_row4_exit_turn_yaw_rad", p.row4_exit_turn_yaw_rad);
  p.turn_kp =
      node.declare_parameter<double>("mf_preselect_turn_kp", p.turn_kp);
  p.turn_max_speed_radps = node.declare_parameter<double>(
      "mf_preselect_turn_max_speed_radps", p.turn_max_speed_radps);
  p.turn_tolerance_deg = node.declare_parameter<double>(
      "mf_preselect_turn_tolerance_deg", p.turn_tolerance_deg);
  p.turn_stable_ticks = node.declare_parameter<int>(
      "mf_preselect_turn_stable_ticks", p.turn_stable_ticks);
  p.turn_timeout_s = node.declare_parameter<double>(
      "mf_preselect_turn_timeout_s", p.turn_timeout_s);
  p.odom_timeout_s = node.declare_parameter<double>(
      "mf_preselect_odom_timeout_s", p.odom_timeout_s);
  p.heading_kp = node.declare_parameter<double>(
      "mf_preselect_heading_kp", p.heading_kp);
  p.heading_max_speed_radps = node.declare_parameter<double>(
      "mf_preselect_heading_max_speed_radps", p.heading_max_speed_radps);
  p.path_r1_lost_wait_timeout_s = node.declare_parameter<double>(
      "mf_preselect_path_r1_lost_wait_timeout_s",
      p.path_r1_lost_wait_timeout_s);
  p.final_exit_center_offset_m = node.declare_parameter<double>(
      "mf_preselect_final_exit_center_offset_m",
      p.final_exit_center_offset_m);
  p.exit_yaw_rad *= static_cast<double>(mirror_sign);
  p.stair1_direction_yaw_rad *= static_cast<double>(mirror_sign);
  p.stair3_direction_yaw_rad *= static_cast<double>(mirror_sign);
  p.row_scan_left_yaw_delta_rad *= static_cast<double>(mirror_sign);
  p.row_scan_back_yaw_delta_rad *= static_cast<double>(mirror_sign);
  p.row4_exit_turn_yaw_rad *= static_cast<double>(mirror_sign);

  p.vision_config_file = resolveVisionConfig(p.vision_config_file);
  blackboard->set("mf_preselection_params", p);

  // 入口 2 号相对分段导航是 XML 前置的可选阶段：这些黑板键供
  // mf_preselection_tree.xml 决定是否先用 odom 闭环到达入口预备姿态。
  const bool entry_nav_enable = node.declare_parameter<bool>(
      "mf_preselect_entry2_nav_enable", false);
  blackboard->set("mf_preselect_entry2_nav_enable", entry_nav_enable);
  blackboard->set(
      "mf_preselect_entry2_nav_segment1_x_m",
      node.declare_parameter<double>("mf_preselect_entry2_nav_segment1_x_m",
                                     0.0));
  const double entry2_nav_segment1_y_m =
      node.declare_parameter<double>("mf_preselect_entry2_nav_segment1_y_m",
                                     0.0) *
      static_cast<double>(mirror_sign);
  blackboard->set(
      "mf_preselect_entry2_nav_segment1_y_m", entry2_nav_segment1_y_m);
  blackboard->set(
      "mf_preselect_entry2_nav_timeout_sec",
      node.declare_parameter<double>("mf_preselect_entry2_nav_timeout_sec",
                                     180.0));

  const bool mc_to_mf_nav_enable = node.declare_parameter<bool>(
      "mc_to_mf_preselect_nav_enable", entry_nav_enable);
  blackboard->set("mc_to_mf_preselect_nav_enable", mc_to_mf_nav_enable);
  blackboard->set(
      "mc_to_mf_preselect_nav_segment1_x_m",
      node.declare_parameter<double>("mc_to_mf_preselect_nav_segment1_x_m",
                                     -2.4));
  const double mc_to_mf_nav_turn_delta_rad =
      node.declare_parameter<double>("mc_to_mf_preselect_nav_turn_delta_rad",
                                     -1.5707963267948966) *
      static_cast<double>(mirror_sign);
  blackboard->set(
      "mc_to_mf_preselect_nav_turn_delta_rad", mc_to_mf_nav_turn_delta_rad);
  blackboard->set("mc_to_mf_preselect_nav_turn_target_yaw", 0.0);
  blackboard->set(
      "mc_to_mf_preselect_nav_segment2_x_m",
      node.declare_parameter<double>("mc_to_mf_preselect_nav_segment2_x_m",
                                     1.6));
  blackboard->set(
      "mc_to_mf_preselect_nav_timeout_sec",
      node.declare_parameter<double>("mc_to_mf_preselect_nav_timeout_sec",
                                     180.0));

  RCLCPP_INFO(node.get_logger(),
              "梅林预选赛参数已加载: model=%s mirror_sign=%d R2_prefixes=%zu fake_prefixes=%zu max_pickup=%d cmd_vel=%s odom=%s entry_y=%.3f mc_to_mf_turn=%.3f exit_yaw=%.3f row4_yaw=%.3f entry_grab_up=0x%02X done=0x%02X high_raise=0x%02X done=0x%02X second_lower=0x%02X done=0x%02X",
              p.model_id.c_str(), mirror_sign,
              p.r2_target_label_prefixes.size(), p.fake_label_prefixes.size(),
              p.max_pickup_count, p.cmd_vel_topic.c_str(),
              p.odom_topic.c_str(), entry2_nav_segment1_y_m,
              mc_to_mf_nav_turn_delta_rad,
              p.exit_yaw_rad, p.row4_exit_turn_yaw_rad,
              static_cast<unsigned int>(std::clamp(p.entry_grab_kfs_up_command_id, 0, 255)),
              static_cast<unsigned int>(std::clamp(p.entry_grab_kfs_up_done_feedback_id, 0, 255)),
              static_cast<unsigned int>(std::clamp(p.arm_high_raise_command_id, 0, 255)),
              static_cast<unsigned int>(std::clamp(p.arm_high_raise_done_feedback_id, 0, 255)),
              static_cast<unsigned int>(std::clamp(p.second_arm_lower_command_id, 0, 255)),
              static_cast<unsigned int>(std::clamp(p.second_arm_lower_done_feedback_id, 0, 255)));
}

void registerMfPreselectionNodes(BT::BehaviorTreeFactory &factory) {
  factory.registerNodeType<MfPreselectionFlowAction>("MfPreselectionFlow");
}

} // namespace rc26_decision
