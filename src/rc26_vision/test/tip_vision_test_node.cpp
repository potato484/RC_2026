// Combined tip vision test node implementation. This file is intentionally
// kept under package-root test/ so the test-only node does not publish
// private headers through include/.

#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <opencv2/opencv.hpp>
#include <rc26_interfaces/msg/mechanism_transport_feedback.hpp>
#include <rc26_interfaces/srv/send_mechanism_transport_command.hpp>

#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <filesystem>
#include <future>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "rc26_serial/protocol.hpp"
#include "rc26_vision/inference/config/model_profile.hpp"
#include "rc26_vision/inference/contracts/inference_engine.hpp"
#include "rc26_vision/postprocess/alignment/tip_alignment.hpp"

namespace rc26_vision::test {

constexpr uint8_t kDefaultGrabTipCommand =
  static_cast<uint8_t>(rc26_serial::CommandID::GRAB_TIP);

class TipVisionTestNode : public rclcpp::Node
{
public:
  TipVisionTestNode();
  ~TipVisionTestNode() override;

  int run();

private:
  using TwistMsg = geometry_msgs::msg::Twist;
  using OdomMsg = nav_msgs::msg::Odometry;
  using FeedbackMsg = rc26_interfaces::msg::MechanismTransportFeedback;
  using SendCommandSrv = rc26_interfaces::srv::SendMechanismTransportCommand;

  enum class AlignmentGrabPhase
  {
    WaitingForAlignment,
    ApproachingLimit,
    SendingGrab,
    Sent,
    Failed,
  };

  struct AlignmentOverlayInfo
  {
    bool control_enabled{false};
    bool grab_enabled{false};
    bool has_target{false};
    bool aligned{false};
    bool command_published{false};
    std::string label{"LOST"};
    int offset_px{0};
    int tolerance_px{20};
    int stable_count{0};
    int stable_required{3};
    int lost_count{0};
    bool target_locked{false};
    int target_lock_lost_count{0};
    double cmd_vx{0.0};
    double cmd_vy{0.0};
    double cmd_wz{0.0};
    bool heading_enabled{false};
    bool heading_stale{false};
    bool heading_aligned{true};
    bool heading_within_gate{true};
    double heading_error_rad{0.0};
    std::string grab_state{"DISABLED"};
    bool limit_switch_triggered{false};
    std::string grab_phase{"WAIT"};
    uint8_t grab_command_id{kDefaultGrabTipCommand};
    uint8_t grab_seq{0x00};
  };

  using TargetCandidate = rc26_vision::TipTargetCandidate;

  struct InferenceResult
  {
    bool valid{false};
    uint64_t inference_seq{0};
    uint64_t source_frame_seq{0};
    std::vector<rc26_vision::Detection> detections;
    std::optional<TargetCandidate> primary_target;
    bool has_target{false};
    int box_w{0};
    int box_cx{-1};
    int offset_px{0};
    int class_id{-1};
    std::size_t detection_count{0U};
    bool target_locked{false};
    int target_lock_lost_count{0};
    double infer_ms{0.0};
    std::chrono::steady_clock::time_point updated_tp{};
  };

  void declare_parameters();
  void load_parameters();
  std::string resolve_resource_path(
    const std::string & configured_path,
    const std::filesystem::path & package_relative_default) const;
  bool resolve_target_class_ids();
  std::string class_id_to_label(int class_id) const;
  rc26_vision::TipAlignmentConfig make_alignment_config() const;

  void create_alignment_interfaces();
  bool start_callback_executor();
  void stop_callback_executor();
  void handle_limit_switch_feedback(const FeedbackMsg::SharedPtr msg);
  void handle_alignment_odom(const OdomMsg::SharedPtr msg);
  bool should_publish_alignment_command(
    const std::chrono::steady_clock::time_point & now,
    bool force) const;
  bool publish_alignment_command(double vx, double vy, double wz, bool force = false);
  void publish_alignment_stop(bool force = false);
  double compute_alignment_vy(int offset_px) const;
  bool read_alignment_yaw(double & yaw_rad, double & age_s);
  rc26_vision::TipHeadingControl compute_alignment_heading_control(
    bool & stale,
    double & yaw_age_s);
  double compute_approach_vx() const;
  std::string grab_phase_label() const;
  void begin_limit_switch_approach();
  std::string update_alignment_control(
    bool has_target,
    int offset_px,
    bool new_inference_result,
    double & out_cmd_vx,
    double & out_cmd_vy,
    bool & out_aligned,
    bool & out_command_published,
    double & out_cmd_wz,
    bool & out_heading_stale,
    bool & out_heading_aligned,
    bool & out_heading_within_gate,
    double & out_heading_error_rad,
    uint8_t & out_grab_seq);
  bool send_grab_tip_command(uint8_t & out_seq);

  bool run_inference_on_frame(
    const cv::Mat & frame_bgr, uint64_t source_frame_seq,
    InferenceResult & result);
  void submit_async_frame(const cv::Mat & frame_bgr, uint64_t source_frame_seq);
  bool copy_latest_inference_result(InferenceResult & result);
  void async_inference_worker_loop();
  bool start_async_inference_worker();
  void stop_async_inference_worker();

  bool initialize();
  void configure_camera_properties();
  bool open_camera_by_index(int index, std::string & opened_source);
  bool open_camera_by_path(const std::string & path, std::string & opened_source);
  std::vector<std::string> discover_video_devices() const;
  int parse_video_index_from_path(const std::string & path) const;
  bool init_camera();
  bool init_inference();
  void draw_alignment_guides(
    cv::Mat & frame_bgr,
    bool has_target,
    int box_cx,
    bool aligned) const;
  void draw_alignment_overlay(cv::Mat & frame_bgr, const AlignmentOverlayInfo & info) const;
  void draw_detections(
    cv::Mat & frame_bgr, const std::vector<rc26_vision::Detection> & detections) const;

  std::string vision_config_file_;
  std::string model_id_{"tip_default"};
  rc26_vision::ModelProfile model_profile_;
  std::filesystem::path package_share_dir_;
  int camera_index_{0};
  std::string camera_device_;
  bool auto_scan_camera_{true};
  std::string selected_camera_source_;
  int camera_width_{640};
  int camera_height_{480};
  int camera_fps_{30};
  bool single_target_mode_{true};
  int max_categories_{5};
  std::string target_name_{"R_R1"};
  int target_class_id_{-1};
  bool use_predicted_label_{false};
  bool show_center_distance_{true};
  double target_real_width_m_{0.027};
  double target_real_height_m_{0.0};
  double camera_fx_px_{0.0};
  double camera_fy_px_{0.0};
  double camera_hfov_deg_{70.0};
  double camera_vfov_deg_{55.0};
  double distance_min_box_px_{8.0};
  double distance_offset_m_{0.0};
  bool show_window_{false};
  std::string window_name_{"Rhino X1 Vision - R_R1"};
  double log_interval_sec_{2.0};
  bool async_inference_{false};
  std::vector<std::string> target_labels_{"D_0", "D_1"};
  std::vector<int> target_class_ids_;
  bool alignment_control_enable_{false};
  std::string alignment_cmd_vel_topic_{"cmd_vel"};
  int alignment_tolerance_px_{20};
  int alignment_stable_frames_{3};
  double alignment_kp_{0.0015};
  double alignment_min_speed_mps_{0.04};
  double alignment_max_speed_mps_{0.15};
  double alignment_command_rate_hz_{20.0};
  int alignment_lost_stop_frames_{3};
  bool alignment_target_lock_enable_{true};
  int alignment_target_lock_max_jump_px_{160};
  bool alignment_publish_zero_on_disable_{true};
  bool alignment_invert_direction_{true};
  bool alignment_draw_guides_{true};
  bool alignment_grab_enable_{true};
  int alignment_grab_command_id_{kDefaultGrabTipCommand};
  std::vector<uint8_t> alignment_grab_payload_;
  bool alignment_grab_once_per_target_{true};
  double alignment_grab_cooldown_s_{2.0};
  std::string alignment_grab_service_name_{"/mechanism/send_command"};
  int alignment_grab_service_timeout_ms_{200};
  std::string alignment_limit_switch_feedback_topic_{"/mechanism/command_feedback"};
  int alignment_limit_switch_feedback_id_{
    static_cast<int>(rc26_serial::FeedbackID::FRONT_LIMIT_SWITCH_TRIGGERED)};
  double alignment_approach_speed_mps_{0.04};
  double alignment_approach_timeout_s_{5.0};
  bool alignment_heading_hold_enable_{true};
  std::string alignment_odom_topic_{"odom"};
  double alignment_target_yaw_rad_{-1.4857};
  double alignment_heading_kp_{1.2};
  double alignment_heading_max_speed_radps_{0.30};
  double alignment_heading_tolerance_rad_{0.05235987755982989};
  double alignment_heading_gate_rad_{0.13962634015954636};
  double alignment_odom_timeout_s_{0.5};

  rc26_vision::InferenceEnginePtr engine_;
  cv::VideoCapture camera_;
  rclcpp::Publisher<TwistMsg>::SharedPtr alignment_cmd_pub_;
  rclcpp::Client<SendCommandSrv>::SharedPtr grab_command_client_;
  rclcpp::Subscription<FeedbackMsg>::SharedPtr limit_switch_sub_;
  rclcpp::Subscription<OdomMsg>::SharedPtr alignment_odom_sub_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> callback_executor_;
  std::thread callback_executor_thread_;
  std::atomic<bool> callback_executor_running_{false};

  std::vector<std::string> class_names_;
  std::mutex async_mutex_;
  std::condition_variable async_cv_;
  std::thread async_worker_thread_;
  cv::Mat async_pending_frame_;
  uint64_t async_pending_frame_seq_{0U};
  bool async_has_pending_frame_{false};
  bool async_stop_requested_{false};
  std::mutex inference_result_mutex_;
  InferenceResult latest_inference_result_;
  bool has_inference_result_{false};
  uint64_t inference_result_seq_{0U};
  int alignment_stable_count_{0};
  int alignment_lost_count_{0};
  AlignmentGrabPhase alignment_grab_phase_{AlignmentGrabPhase::WaitingForAlignment};
  bool grab_sent_for_current_target_{false};
  std::string last_grab_state_{"DISABLED"};
  uint8_t last_grab_seq_{0x00};
  std::chrono::steady_clock::time_point last_alignment_command_tp_{};
  std::chrono::steady_clock::time_point last_grab_attempt_tp_{};
  std::chrono::steady_clock::time_point limit_switch_wait_start_tp_{};
  bool alignment_zero_published_{false};
  std::atomic<bool> waiting_for_limit_switch_{false};
  std::atomic<bool> limit_switch_triggered_{false};
  std::mutex alignment_odom_mutex_;
  bool alignment_has_yaw_{false};
  double alignment_current_yaw_rad_{0.0};
  std::chrono::steady_clock::time_point alignment_odom_receive_tp_{};
  std::mutex grab_response_mutex_;
  std::condition_variable grab_response_cv_;
  std::atomic<uint64_t> grab_request_generation_{0};
  bool grab_response_seen_{false};
  bool grab_response_accepted_{false};
  uint8_t grab_response_seq_{0U};
  rc26_vision::TipTargetLockState alignment_target_lock_state_;

  uint64_t frames_since_log_{0};
  std::chrono::steady_clock::time_point last_log_tp_{};
};

}  // namespace rc26_vision::test

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <algorithm>
#include <chrono>
#include <cctype>
#include <cstdint>
#include <filesystem>
#include <iomanip>
#include <sstream>
#include <string>

namespace rc26_vision::test::tip_detail {

inline std::string trim_copy(const std::string & value)
{
  const std::string whitespace = " \t\r\n";
  const std::size_t begin = value.find_first_not_of(whitespace);
  if (begin == std::string::npos) {
    return "";
  }
  const std::size_t end = value.find_last_not_of(whitespace);
  return value.substr(begin, end - begin + 1U);
}

inline std::filesystem::path get_rc26_vision_share_dir()
{
  try {
    return ament_index_cpp::get_package_share_directory("rc26_vision");
  } catch (...) {
    return {};
  }
}

template<typename ByteContainer>
inline std::string bytes_to_hex(const ByteContainer & bytes)
{
  std::ostringstream oss;
  oss << std::uppercase << std::hex << std::setfill('0');
  for (std::size_t i = 0; i < bytes.size(); ++i) {
    if (i > 0U) {
      oss << " ";
    }
    oss << std::setw(2) << static_cast<int>(bytes[i]);
  }
  return oss.str();
}

inline std::string byte_to_hex(uint8_t value)
{
  std::ostringstream oss;
  oss << "0x" << std::uppercase << std::hex << std::setfill('0') << std::setw(2)
      << static_cast<unsigned int>(value);
  return oss.str();
}

inline double yaw_from_quaternion(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(
    2.0 * (q.w * q.z + q.x * q.y),
    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

}  // namespace rc26_vision::test::tip_detail

// ---- tip_vision_test_node.cpp ----

#include <chrono>
#include <cstdio>
#include <iomanip>
#include <sstream>

namespace rc26_vision::test {

using namespace std::chrono_literals;

TipVisionTestNode::TipVisionTestNode()
: Node("tip_vision_test_node")
{
  package_share_dir_ = tip_detail::get_rc26_vision_share_dir();
  declare_parameters();
  load_parameters();
}

TipVisionTestNode::~TipVisionTestNode()
{
  stop_callback_executor();
  stop_async_inference_worker();

  engine_.reset();

  if (camera_.isOpened()) {
    camera_.release();
  }

  if (show_window_) {
    cv::destroyAllWindows();
  }
}

int TipVisionTestNode::run()
{
  if (!initialize()) {
    return 1;
  }

  if (show_window_) {
    cv::namedWindow(window_name_, cv::WINDOW_NORMAL);
    cv::resizeWindow(window_name_, camera_width_, camera_height_);
  }

  if (async_inference_ && !start_async_inference_worker()) {
    return 1;
  }

  frames_since_log_ = 0;
  last_log_tp_ = std::chrono::steady_clock::now();
  auto last_perf_sample_tp = last_log_tp_;
  auto last_capture_warn_tp = std::chrono::steady_clock::now();
  uint64_t source_frame_seq = 0U;
  uint64_t last_applied_inference_seq = 0U;
  uint64_t last_logged_inference_seq = 0U;
  uint64_t last_perf_source_frame_seq = 0U;
  uint64_t last_perf_inference_seq = 0U;
  InferenceResult latest_result;
  bool has_latest_result = false;
  bool current_has_target = false;
  int current_box_cx = -1;
  int current_offset_px = 0;
  int current_class_id = -1;
  std::size_t current_detection_count = 0U;
  bool current_target_locked = false;
  int current_target_lock_lost_count = 0;
  double current_infer_ms = 0.0;
  double display_camera_fps = 0.0;
  double display_infer_fps = 0.0;
  double current_cmd_vx = 0.0;
  double current_cmd_vy = 0.0;
  double current_cmd_wz = 0.0;
  bool current_aligned = false;
  bool current_heading_stale = false;
  bool current_heading_aligned = true;
  bool current_heading_within_gate = true;
  double current_heading_error_rad = 0.0;
  bool alignment_command_published = false;

  while (rclcpp::ok()) {
    cv::Mat frame_bgr;
    if (!camera_.read(frame_bgr) || frame_bgr.empty()) {
      const auto now = std::chrono::steady_clock::now();
      if (now - last_capture_warn_tp > 2s) {
        RCLCPP_WARN(get_logger(), "相机帧抓取失败，正在重试...");
        last_capture_warn_tp = now;
      }
      std::this_thread::sleep_for(50ms);
      continue;
    }

    ++source_frame_seq;
    if (async_inference_) {
      submit_async_frame(frame_bgr, source_frame_seq);
    } else {
      InferenceResult sync_result;
      if (!run_inference_on_frame(frame_bgr, source_frame_seq, sync_result)) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000, "共享引擎推理失败。");
        continue;
      }
      latest_inference_result_ = sync_result;
      has_inference_result_ = true;
    }

    bool new_inference_result = false;
    if (copy_latest_inference_result(latest_result)) {
      has_latest_result = true;
      current_detection_count = latest_result.detection_count;
      current_infer_ms = latest_result.infer_ms;

      if (latest_result.inference_seq != last_applied_inference_seq) {
        new_inference_result = true;
        last_applied_inference_seq = latest_result.inference_seq;
        current_has_target = latest_result.has_target;
        current_target_locked = latest_result.target_locked;
        current_target_lock_lost_count = latest_result.target_lock_lost_count;
        if (latest_result.has_target) {
          current_box_cx = latest_result.box_cx;
          current_offset_px = latest_result.offset_px;
          current_class_id = latest_result.class_id;
        } else {
          current_box_cx = -1;
          current_offset_px = 0;
          current_class_id = -1;
        }
      }
    }

    uint8_t grab_seq = last_grab_seq_;
    const std::string grab_state = update_alignment_control(
      current_has_target, current_offset_px, new_inference_result, current_cmd_vx, current_cmd_vy,
      current_aligned, alignment_command_published, current_cmd_wz, current_heading_stale,
      current_heading_aligned, current_heading_within_gate, current_heading_error_rad, grab_seq);

    if (show_window_ && has_latest_result) {
      draw_detections(frame_bgr, latest_result.detections);
    }

    const auto frame_end = std::chrono::steady_clock::now();
    const uint64_t current_inference_seq =
      has_latest_result ? latest_result.inference_seq : last_applied_inference_seq;
    const double perf_elapsed =
      std::chrono::duration<double>(frame_end - last_perf_sample_tp).count();
    if (perf_elapsed >= 0.5) {
      display_camera_fps =
        static_cast<double>(source_frame_seq - last_perf_source_frame_seq) /
        std::max(1e-6, perf_elapsed);
      display_infer_fps =
        static_cast<double>(current_inference_seq - last_perf_inference_seq) /
        std::max(1e-6, perf_elapsed);
      last_perf_sample_tp = frame_end;
      last_perf_source_frame_seq = source_frame_seq;
      last_perf_inference_seq = current_inference_seq;
    }

    if (show_window_) {
      draw_alignment_guides(frame_bgr, current_has_target, current_box_cx, current_aligned);

      AlignmentOverlayInfo alignment_info;
      alignment_info.control_enabled = alignment_control_enable_;
      alignment_info.grab_enabled = alignment_control_enable_ && alignment_grab_enable_;
      alignment_info.has_target = current_has_target;
      alignment_info.aligned = current_aligned;
      alignment_info.command_published = alignment_command_published;
      alignment_info.label = class_id_to_label(current_class_id);
      alignment_info.offset_px = current_offset_px;
      alignment_info.tolerance_px = alignment_tolerance_px_;
      alignment_info.stable_count = alignment_stable_count_;
      alignment_info.stable_required = alignment_stable_frames_;
      alignment_info.lost_count = alignment_lost_count_;
      alignment_info.target_locked = current_target_locked;
      alignment_info.target_lock_lost_count = current_target_lock_lost_count;
      alignment_info.cmd_vx = current_cmd_vx;
      alignment_info.cmd_vy = current_cmd_vy;
      alignment_info.cmd_wz = current_cmd_wz;
      alignment_info.heading_enabled = alignment_heading_hold_enable_;
      alignment_info.heading_stale = current_heading_stale;
      alignment_info.heading_aligned = current_heading_aligned;
      alignment_info.heading_within_gate = current_heading_within_gate;
      alignment_info.heading_error_rad = current_heading_error_rad;
      alignment_info.grab_state = grab_state;
      alignment_info.limit_switch_triggered =
        limit_switch_triggered_.load(std::memory_order_relaxed);
      alignment_info.grab_phase = grab_phase_label();
      alignment_info.grab_command_id = static_cast<uint8_t>(alignment_grab_command_id_);
      alignment_info.grab_seq = grab_seq;
      draw_alignment_overlay(frame_bgr, alignment_info);

      std::ostringstream perf_ss;
      perf_ss << "FPS:" << std::fixed << std::setprecision(1) << display_infer_fps
              << " Cam:" << std::setprecision(1) << display_camera_fps
              << " Infer:" << std::setprecision(2) << current_infer_ms << "ms"
              << " Det:" << current_detection_count;
      cv::putText(
        frame_bgr, perf_ss.str(), cv::Point(12, 28),
        cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(50, 220, 50), 2, cv::LINE_AA);
      cv::imshow(window_name_, frame_bgr);
      const int key = cv::waitKey(1) & 0xFF;
      if (key == 'q' || key == 27) {
        RCLCPP_INFO(get_logger(), "键盘请求退出。");
        break;
      }
    }

    ++frames_since_log_;
    const auto now = std::chrono::steady_clock::now();
    const double elapsed = std::chrono::duration<double>(now - last_log_tp_).count();
    if (elapsed >= log_interval_sec_) {
      const double loop_fps =
        static_cast<double>(frames_since_log_) / std::max(1e-6, elapsed);
      const double infer_fps =
        static_cast<double>(current_inference_seq - last_logged_inference_seq) /
        std::max(1e-6, elapsed);
      RCLCPP_INFO(
        get_logger(), "循环帧率=%.2f 推理帧率=%.2f 推理耗时=%.2fms 检测数=%zu",
        loop_fps, infer_fps, current_infer_ms, current_detection_count);
      frames_since_log_ = 0;
      last_log_tp_ = now;
      last_logged_inference_seq = current_inference_seq;
    }
  }

  stop_async_inference_worker();
  stop_callback_executor();
  publish_alignment_stop(true);
  return 0;
}

}  // namespace rc26_vision::test

// ---- tip_vision_test_node_params.cpp ----

#include <algorithm>
#include <cmath>
#include <sstream>
#include <stdexcept>
#include <unordered_set>

namespace rc26_vision::test {

void TipVisionTestNode::declare_parameters()
{
  const std::string default_config_file =
    package_share_dir_.empty() ?
    "config/vision_models.yaml" :
    (package_share_dir_ / "config" / "vision_models.yaml").string();

  this->declare_parameter<std::string>("vision_config_file", default_config_file);
  this->declare_parameter<std::string>("model_id", "tip_default");
  this->declare_parameter<int>("camera_index", 0);
  this->declare_parameter<std::string>("camera_device", "");
  this->declare_parameter<bool>("auto_scan_camera", true);
  this->declare_parameter<int>("width", 640);
  this->declare_parameter<int>("height", 480);
  this->declare_parameter<int>("fps", 30);
  this->declare_parameter<bool>("single_target_mode", false);
  this->declare_parameter<int>("max_categories", 5);
  this->declare_parameter<std::string>("target_name", "JK");
  this->declare_parameter<int>("target_class_id", -1);
  this->declare_parameter<bool>("use_predicted_label", true);
  this->declare_parameter<bool>("show_center_distance", false);
  this->declare_parameter<double>("target_real_width_m", 0.027);
  this->declare_parameter<double>("target_real_height_m", 0.0);
  this->declare_parameter<double>("camera_fx_px", 0.0);
  this->declare_parameter<double>("camera_fy_px", 0.0);
  this->declare_parameter<double>("camera_hfov_deg", 70.0);
  this->declare_parameter<double>("camera_vfov_deg", 55.0);
  this->declare_parameter<double>("distance_min_box_px", 8.0);
  this->declare_parameter<double>("distance_offset_m", 0.0);
  this->declare_parameter<bool>("show_window", false);
  this->declare_parameter<std::string>("window_name", "Rhino X1 Vision - V2 Class Check");
  this->declare_parameter<double>("log_interval_sec", 2.0);
  this->declare_parameter<bool>("async_inference", false);
  this->declare_parameter<std::vector<std::string>>(
    "target_labels", std::vector<std::string>{"JK"});
  this->declare_parameter<bool>("alignment_control_enable", false);
  this->declare_parameter<std::string>("alignment_cmd_vel_topic", "cmd_vel");
  this->declare_parameter<int>("alignment_tolerance_px", 20);
  this->declare_parameter<int>("alignment_stable_frames", 3);
  this->declare_parameter<double>("alignment_kp", 0.0015);
  this->declare_parameter<double>("alignment_min_speed_mps", 0.04);
  this->declare_parameter<double>("alignment_max_speed_mps", 0.15);
  this->declare_parameter<double>("alignment_command_rate_hz", 20.0);
  this->declare_parameter<int>("alignment_lost_stop_frames", 3);
  this->declare_parameter<bool>("alignment_target_lock_enable", true);
  this->declare_parameter<int>("alignment_target_lock_max_jump_px", 160);
  this->declare_parameter<bool>("alignment_publish_zero_on_disable", true);
  this->declare_parameter<bool>("alignment_invert_direction", true);
  this->declare_parameter<bool>("alignment_draw_guides", true);
  this->declare_parameter<bool>("alignment_grab_enable", true);
  this->declare_parameter<int>("alignment_grab_command_id", kDefaultGrabTipCommand);
  this->declare_parameter<std::vector<int64_t>>(
    "alignment_grab_payload", std::vector<int64_t>{});
  this->declare_parameter<bool>("alignment_grab_once_per_target", true);
  this->declare_parameter<double>("alignment_grab_cooldown_s", 2.0);
  this->declare_parameter<std::string>("alignment_grab_service_name", "/mechanism/send_command");
  this->declare_parameter<int>("alignment_grab_service_timeout_ms", 200);
  this->declare_parameter<std::string>(
    "alignment_limit_switch_feedback_topic", "/mechanism/command_feedback");
  this->declare_parameter<int>(
    "alignment_limit_switch_feedback_id",
    static_cast<int>(rc26_serial::FeedbackID::FRONT_LIMIT_SWITCH_TRIGGERED));
  this->declare_parameter<double>("alignment_approach_speed_mps", 0.04);
  this->declare_parameter<double>("alignment_approach_timeout_s", 5.0);
  this->declare_parameter<bool>("alignment_heading_hold_enable", true);
  this->declare_parameter<std::string>("alignment_odom_topic", "odom");
  this->declare_parameter<double>("alignment_target_yaw_rad", -1.4857);
  this->declare_parameter<double>("alignment_heading_kp", 1.2);
  this->declare_parameter<double>("alignment_heading_max_speed_radps", 0.30);
  this->declare_parameter<double>("alignment_heading_tolerance_deg", 3.0);
  this->declare_parameter<double>("alignment_heading_gate_deg", 8.0);
  this->declare_parameter<double>("alignment_odom_timeout_s", 0.5);
}

void TipVisionTestNode::load_parameters()
{
  constexpr double kDeg2Rad = 3.14159265358979323846 / 180.0;

  vision_config_file_ = this->get_parameter("vision_config_file").as_string();
  model_id_ = this->get_parameter("model_id").as_string();
  camera_index_ = this->get_parameter("camera_index").as_int();
  camera_device_ = this->get_parameter("camera_device").as_string();
  auto_scan_camera_ = this->get_parameter("auto_scan_camera").as_bool();
  camera_width_ = this->get_parameter("width").as_int();
  camera_height_ = this->get_parameter("height").as_int();
  camera_fps_ = this->get_parameter("fps").as_int();
  single_target_mode_ = this->get_parameter("single_target_mode").as_bool();
  max_categories_ = this->get_parameter("max_categories").as_int();
  target_name_ = this->get_parameter("target_name").as_string();
  target_class_id_ = this->get_parameter("target_class_id").as_int();
  use_predicted_label_ = this->get_parameter("use_predicted_label").as_bool();
  show_center_distance_ = this->get_parameter("show_center_distance").as_bool();
  target_real_width_m_ = this->get_parameter("target_real_width_m").as_double();
  target_real_height_m_ = this->get_parameter("target_real_height_m").as_double();
  camera_fx_px_ = this->get_parameter("camera_fx_px").as_double();
  camera_fy_px_ = this->get_parameter("camera_fy_px").as_double();
  camera_hfov_deg_ = this->get_parameter("camera_hfov_deg").as_double();
  camera_vfov_deg_ = this->get_parameter("camera_vfov_deg").as_double();
  distance_min_box_px_ = this->get_parameter("distance_min_box_px").as_double();
  distance_offset_m_ = this->get_parameter("distance_offset_m").as_double();
  show_window_ = this->get_parameter("show_window").as_bool();
  window_name_ = this->get_parameter("window_name").as_string();
  log_interval_sec_ = this->get_parameter("log_interval_sec").as_double();
  async_inference_ = this->get_parameter("async_inference").as_bool();
  target_labels_ = this->get_parameter("target_labels").as_string_array();
  alignment_control_enable_ = this->get_parameter("alignment_control_enable").as_bool();
  alignment_cmd_vel_topic_ = this->get_parameter("alignment_cmd_vel_topic").as_string();
  alignment_tolerance_px_ = this->get_parameter("alignment_tolerance_px").as_int();
  alignment_stable_frames_ = this->get_parameter("alignment_stable_frames").as_int();
  alignment_kp_ = this->get_parameter("alignment_kp").as_double();
  alignment_min_speed_mps_ = this->get_parameter("alignment_min_speed_mps").as_double();
  alignment_max_speed_mps_ = this->get_parameter("alignment_max_speed_mps").as_double();
  alignment_command_rate_hz_ = this->get_parameter("alignment_command_rate_hz").as_double();
  alignment_lost_stop_frames_ = this->get_parameter("alignment_lost_stop_frames").as_int();
  alignment_target_lock_enable_ =
    this->get_parameter("alignment_target_lock_enable").as_bool();
  alignment_target_lock_max_jump_px_ =
    this->get_parameter("alignment_target_lock_max_jump_px").as_int();
  alignment_publish_zero_on_disable_ =
    this->get_parameter("alignment_publish_zero_on_disable").as_bool();
  alignment_invert_direction_ = this->get_parameter("alignment_invert_direction").as_bool();
  alignment_draw_guides_ = this->get_parameter("alignment_draw_guides").as_bool();
  alignment_grab_enable_ = this->get_parameter("alignment_grab_enable").as_bool();
  alignment_grab_command_id_ = this->get_parameter("alignment_grab_command_id").as_int();
  alignment_grab_once_per_target_ =
    this->get_parameter("alignment_grab_once_per_target").as_bool();
  alignment_grab_cooldown_s_ = this->get_parameter("alignment_grab_cooldown_s").as_double();
  alignment_grab_service_name_ =
    this->get_parameter("alignment_grab_service_name").as_string();
  alignment_grab_service_timeout_ms_ =
    this->get_parameter("alignment_grab_service_timeout_ms").as_int();
  alignment_limit_switch_feedback_topic_ =
    this->get_parameter("alignment_limit_switch_feedback_topic").as_string();
  alignment_limit_switch_feedback_id_ =
    this->get_parameter("alignment_limit_switch_feedback_id").as_int();
  alignment_approach_speed_mps_ =
    this->get_parameter("alignment_approach_speed_mps").as_double();
  alignment_approach_timeout_s_ =
    this->get_parameter("alignment_approach_timeout_s").as_double();
  alignment_heading_hold_enable_ =
    this->get_parameter("alignment_heading_hold_enable").as_bool();
  alignment_odom_topic_ = this->get_parameter("alignment_odom_topic").as_string();
  alignment_target_yaw_rad_ = this->get_parameter("alignment_target_yaw_rad").as_double();
  alignment_heading_kp_ = this->get_parameter("alignment_heading_kp").as_double();
  alignment_heading_max_speed_radps_ =
    this->get_parameter("alignment_heading_max_speed_radps").as_double();
  alignment_heading_tolerance_rad_ =
    this->get_parameter("alignment_heading_tolerance_deg").as_double() * kDeg2Rad;
  alignment_heading_gate_rad_ =
    this->get_parameter("alignment_heading_gate_deg").as_double() * kDeg2Rad;
  alignment_odom_timeout_s_ = this->get_parameter("alignment_odom_timeout_s").as_double();
  const auto grab_payload_values =
    this->get_parameter("alignment_grab_payload").as_integer_array();
  alignment_grab_payload_.clear();
  alignment_grab_payload_.reserve(grab_payload_values.size());
  for (const auto value : grab_payload_values) {
    if (value < 0 || value > 255) {
      throw std::runtime_error("alignment_grab_payload values must be in [0,255]");
    }
    alignment_grab_payload_.push_back(static_cast<uint8_t>(value));
  }

  vision_config_file_ = resolve_resource_path(
    vision_config_file_, std::filesystem::path("config") / "vision_models.yaml");

  if (tip_detail::trim_copy(model_id_).empty()) {
    throw std::runtime_error("model_id cannot be empty");
  }
  if (camera_width_ <= 0 || camera_height_ <= 0 || camera_fps_ <= 0) {
    throw std::runtime_error("width/height/fps must be > 0");
  }
  if (max_categories_ < 0) {
    throw std::runtime_error("max_categories must be >= 0");
  }
  if (target_labels_.empty()) {
    throw std::runtime_error("target_labels must contain at least one label");
  }
  for (const auto & label : target_labels_) {
    if (tip_detail::trim_copy(label).empty()) {
      throw std::runtime_error("target_labels cannot contain empty values");
    }
  }
  if (alignment_control_enable_ && tip_detail::trim_copy(alignment_cmd_vel_topic_).empty()) {
    throw std::runtime_error("alignment_cmd_vel_topic cannot be empty when alignment control is enabled");
  }
  if (alignment_tolerance_px_ < 0) {
    throw std::runtime_error("alignment_tolerance_px must be >= 0");
  }
  if (alignment_stable_frames_ <= 0) {
    throw std::runtime_error("alignment_stable_frames must be > 0");
  }
  if (!std::isfinite(alignment_kp_) || alignment_kp_ < 0.0) {
    throw std::runtime_error("alignment_kp must be finite and >= 0");
  }
  if (!std::isfinite(alignment_min_speed_mps_) || alignment_min_speed_mps_ < 0.0) {
    throw std::runtime_error("alignment_min_speed_mps must be finite and >= 0");
  }
  if (!std::isfinite(alignment_max_speed_mps_) || alignment_max_speed_mps_ < 0.0) {
    throw std::runtime_error("alignment_max_speed_mps must be finite and >= 0");
  }
  if (alignment_min_speed_mps_ > alignment_max_speed_mps_) {
    throw std::runtime_error("alignment_min_speed_mps must be <= alignment_max_speed_mps");
  }
  if (!std::isfinite(alignment_command_rate_hz_) || alignment_command_rate_hz_ <= 0.0) {
    throw std::runtime_error("alignment_command_rate_hz must be finite and > 0");
  }
  if (alignment_lost_stop_frames_ <= 0) {
    throw std::runtime_error("alignment_lost_stop_frames must be > 0");
  }
  if (alignment_target_lock_max_jump_px_ < 0) {
    throw std::runtime_error("alignment_target_lock_max_jump_px must be >= 0");
  }
  if (alignment_grab_command_id_ < 0 || alignment_grab_command_id_ > 255) {
    throw std::runtime_error("alignment_grab_command_id must be in [0,255]");
  }
  if (!std::isfinite(alignment_grab_cooldown_s_) || alignment_grab_cooldown_s_ < 0.0) {
    throw std::runtime_error("alignment_grab_cooldown_s must be finite and >= 0");
  }
  if (alignment_grab_service_timeout_ms_ <= 0) {
    throw std::runtime_error("alignment_grab_service_timeout_ms must be > 0");
  }
  if (alignment_control_enable_ && alignment_grab_enable_ &&
    tip_detail::trim_copy(alignment_grab_service_name_).empty())
  {
    throw std::runtime_error("alignment_grab_service_name cannot be empty when grab is enabled");
  }
  if (alignment_control_enable_ && alignment_grab_enable_ &&
    tip_detail::trim_copy(alignment_limit_switch_feedback_topic_).empty())
  {
    throw std::runtime_error(
      "alignment_limit_switch_feedback_topic cannot be empty when limit switch is enabled");
  }
  if (alignment_limit_switch_feedback_id_ < 0 || alignment_limit_switch_feedback_id_ > 255) {
    throw std::runtime_error("alignment_limit_switch_feedback_id must be in [0,255]");
  }
  if (!std::isfinite(alignment_approach_speed_mps_) || alignment_approach_speed_mps_ < 0.0) {
    throw std::runtime_error("alignment_approach_speed_mps must be finite and >= 0");
  }
  if (!std::isfinite(alignment_approach_timeout_s_) || alignment_approach_timeout_s_ <= 0.0) {
    throw std::runtime_error("alignment_approach_timeout_s must be finite and > 0");
  }
  if (tip_detail::trim_copy(alignment_odom_topic_).empty()) {
    throw std::runtime_error("alignment_odom_topic cannot be empty");
  }
  if (!std::isfinite(alignment_target_yaw_rad_)) {
    throw std::runtime_error("alignment_target_yaw_rad must be finite");
  }
  if (!std::isfinite(alignment_heading_kp_) || alignment_heading_kp_ < 0.0) {
    throw std::runtime_error("alignment_heading_kp must be finite and >= 0");
  }
  if (!std::isfinite(alignment_heading_max_speed_radps_) ||
    alignment_heading_max_speed_radps_ < 0.0)
  {
    throw std::runtime_error("alignment_heading_max_speed_radps must be finite and >= 0");
  }
  if (!std::isfinite(alignment_heading_tolerance_rad_) || alignment_heading_tolerance_rad_ < 0.0) {
    throw std::runtime_error("alignment_heading_tolerance_deg must be finite and >= 0");
  }
  if (!std::isfinite(alignment_heading_gate_rad_) || alignment_heading_gate_rad_ < 0.0) {
    throw std::runtime_error("alignment_heading_gate_deg must be finite and >= 0");
  }
  if (alignment_heading_gate_rad_ < alignment_heading_tolerance_rad_) {
    throw std::runtime_error("alignment_heading_gate_deg must be >= heading tolerance");
  }
  if (!std::isfinite(alignment_odom_timeout_s_) || alignment_odom_timeout_s_ <= 0.0) {
    throw std::runtime_error("alignment_odom_timeout_s must be finite and > 0");
  }
}

std::string TipVisionTestNode::resolve_resource_path(
  const std::string & configured_path,
  const std::filesystem::path & package_relative_default) const
{
  std::vector<std::filesystem::path> candidates;
  const std::filesystem::path configured_fs = tip_detail::trim_copy(configured_path);
  const std::filesystem::path default_fs =
    package_share_dir_.empty() ? package_relative_default : (package_share_dir_ / package_relative_default);

  if (!configured_fs.empty()) {
    candidates.push_back(configured_fs);

    if (configured_fs.is_relative()) {
      candidates.push_back(std::filesystem::current_path() / configured_fs);
      if (!package_share_dir_.empty()) {
        candidates.push_back(package_share_dir_ / configured_fs);
      }
    }
  }

  candidates.push_back(default_fs);

  for (const auto & candidate : candidates) {
    if (!candidate.empty() && std::filesystem::exists(candidate)) {
      return candidate.lexically_normal().string();
    }
  }

  if (!configured_fs.empty()) {
    return configured_fs.lexically_normal().string();
  }
  return default_fs.lexically_normal().string();
}

bool TipVisionTestNode::resolve_target_class_ids()
{
  if (class_names_.empty()) {
    RCLCPP_ERROR(get_logger(), "模型配置 '%s' 没有标签。", model_id_.c_str());
    return false;
  }

  target_class_ids_.clear();
  target_class_ids_.reserve(target_labels_.size());

  std::unordered_set<int> seen_ids;
  for (const auto & configured_label : target_labels_) {
    const std::string label = tip_detail::trim_copy(configured_label);
    auto it = std::find(class_names_.begin(), class_names_.end(), label);
    if (it == class_names_.end()) {
      RCLCPP_ERROR(
        get_logger(), "配置的目标标签 '%s' 在模型配置标签中未找到。",
        label.c_str());
      return false;
    }
    const int class_id = static_cast<int>(std::distance(class_names_.begin(), it));
    if (seen_ids.find(class_id) != seen_ids.end()) {
      RCLCPP_ERROR(
        get_logger(), "配置的目标标签 '%s' 映射到重复的 class_id=%d。",
        label.c_str(), class_id);
      return false;
    }
    seen_ids.insert(class_id);
    target_class_ids_.push_back(class_id);
  }

  std::ostringstream oss;
  for (std::size_t i = 0; i < target_class_ids_.size(); ++i) {
    if (i > 0U) {
      oss << ", ";
    }
    oss << target_labels_[i] << "->" << target_class_ids_[i];
  }
  RCLCPP_INFO(get_logger(), "目标标签映射: %s", oss.str().c_str());
  return true;
}

std::string TipVisionTestNode::class_id_to_label(int class_id) const
{
  if (class_id >= 0 && class_id < static_cast<int>(class_names_.size())) {
    return class_names_[static_cast<std::size_t>(class_id)];
  }
  if (class_id >= 0) {
    return "class_" + std::to_string(class_id);
  }
  return "LOST";
}

rc26_vision::TipAlignmentConfig TipVisionTestNode::make_alignment_config() const
{
  rc26_vision::TipAlignmentConfig config;
  config.target_lock_enable = alignment_target_lock_enable_;
  config.target_lock_max_jump_px = alignment_target_lock_max_jump_px_;
  config.lost_stop_frames = alignment_lost_stop_frames_;
  config.tolerance_px = alignment_tolerance_px_;
  config.kp = alignment_kp_;
  config.min_speed_mps = alignment_min_speed_mps_;
  config.max_speed_mps = alignment_max_speed_mps_;
  config.invert_direction = alignment_invert_direction_;
  config.heading_hold_enable = alignment_heading_hold_enable_;
  config.target_yaw_rad = alignment_target_yaw_rad_;
  config.heading_kp = alignment_heading_kp_;
  config.heading_max_speed_radps = alignment_heading_max_speed_radps_;
  config.heading_tolerance_rad = alignment_heading_tolerance_rad_;
  config.heading_gate_rad = alignment_heading_gate_rad_;
  return config;
}

void TipVisionTestNode::create_alignment_interfaces()
{
  if (!alignment_control_enable_) {
    RCLCPP_INFO(get_logger(), "端头对准控制已禁用；未创建 cmd_vel 发布者或抓取客户端。");
    return;
  }

  alignment_cmd_pub_ = create_publisher<TwistMsg>(alignment_cmd_vel_topic_, rclcpp::QoS(10));
  if (alignment_grab_enable_) {
    grab_command_client_ = create_client<SendCommandSrv>(alignment_grab_service_name_);
  }
  if (alignment_grab_enable_) {
    limit_switch_sub_ = create_subscription<FeedbackMsg>(
      alignment_limit_switch_feedback_topic_, rclcpp::QoS(32).reliable(),
      [this](const FeedbackMsg::SharedPtr msg) { handle_limit_switch_feedback(msg); });
  }
  if (alignment_heading_hold_enable_) {
    alignment_odom_sub_ = create_subscription<OdomMsg>(
      alignment_odom_topic_, rclcpp::QoS(rclcpp::KeepLast(10)),
      [this](const OdomMsg::SharedPtr msg) { handle_alignment_odom(msg); });
  }
  if (!start_callback_executor()) {
    throw std::runtime_error("启动回调 executor 失败");
  }

  RCLCPP_WARN(
    get_logger(),
    "端头对准控制已启用: cmd_vel_topic=%s 容差=%dpx 稳定帧=%d heading=%s odom=%s target_yaw=%.3f 抓取=%s service=%s limit_feedback=%s id=0x%02X",
    alignment_cmd_vel_topic_.c_str(), alignment_tolerance_px_, alignment_stable_frames_,
    alignment_heading_hold_enable_ ? "开" : "关", alignment_odom_topic_.c_str(),
    alignment_target_yaw_rad_,
    alignment_grab_enable_ ? "开" : "关", alignment_grab_service_name_.c_str(),
    alignment_limit_switch_feedback_topic_.c_str(),
    static_cast<unsigned int>(alignment_limit_switch_feedback_id_ & 0xFF));
}

bool TipVisionTestNode::start_callback_executor()
{
  if (callback_executor_running_.load(std::memory_order_relaxed)) {
    return true;
  }

  try {
    callback_executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    callback_executor_->add_node(get_node_base_interface());
    callback_executor_running_ = true;
    callback_executor_thread_ = std::thread([this]() {
      while (rclcpp::ok() && callback_executor_running_.load(std::memory_order_relaxed)) {
        callback_executor_->spin_some();
        std::this_thread::sleep_for(5ms);
      }
    });
    return true;
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(get_logger(), "启动回调 executor 失败: %s", ex.what());
    callback_executor_running_ = false;
    if (callback_executor_) {
      callback_executor_->cancel();
      try {
        callback_executor_->remove_node(get_node_base_interface());
      } catch (...) {
      }
      callback_executor_.reset();
    }
    return false;
  }
}

void TipVisionTestNode::stop_callback_executor()
{
  waiting_for_limit_switch_ = false;
  grab_request_generation_.fetch_add(1, std::memory_order_relaxed);
  grab_response_cv_.notify_all();
  callback_executor_running_ = false;
  if (callback_executor_) {
    callback_executor_->cancel();
  }
  if (callback_executor_thread_.joinable()) {
    callback_executor_thread_.join();
  }
  if (callback_executor_) {
    try {
      callback_executor_->remove_node(get_node_base_interface());
    } catch (...) {
    }
    callback_executor_.reset();
  }
  limit_switch_sub_.reset();
  alignment_odom_sub_.reset();
}

bool TipVisionTestNode::should_publish_alignment_command(
  const std::chrono::steady_clock::time_point & now,
  bool force) const
{
  if (!alignment_cmd_pub_) {
    return false;
  }
  if (force || last_alignment_command_tp_ == std::chrono::steady_clock::time_point{}) {
    return true;
  }
  const double min_period_s = 1.0 / std::max(1e-6, alignment_command_rate_hz_);
  return std::chrono::duration<double>(now - last_alignment_command_tp_).count() >= min_period_s;
}

bool TipVisionTestNode::publish_alignment_command(double vx, double vy, double wz, bool force)
{
  if (!alignment_control_enable_ || !alignment_cmd_pub_) {
    return false;
  }

  const auto now = std::chrono::steady_clock::now();
  if (!should_publish_alignment_command(now, force)) {
    return false;
  }

  TwistMsg msg;
  msg.linear.x = vx;
  msg.linear.y = vy;
  msg.angular.z = wz;
  alignment_cmd_pub_->publish(msg);
  last_alignment_command_tp_ = now;
  alignment_zero_published_ =
    std::abs(vx) < 1e-9 && std::abs(vy) < 1e-9 && std::abs(wz) < 1e-9;
  return true;
}

void TipVisionTestNode::publish_alignment_stop(bool force)
{
  if (!alignment_publish_zero_on_disable_ && !alignment_control_enable_) {
    return;
  }
  publish_alignment_command(0.0, 0.0, 0.0, force);
}

double TipVisionTestNode::compute_alignment_vy(int offset_px) const
{
  return rc26_vision::computeTipAlignmentVy(offset_px, make_alignment_config());
}

double TipVisionTestNode::compute_approach_vx() const
{
  return rc26_vision::computeTipApproachVx(alignment_approach_speed_mps_);
}

std::string TipVisionTestNode::grab_phase_label() const
{
  switch (alignment_grab_phase_) {
    case AlignmentGrabPhase::WaitingForAlignment:
      return "WAIT";
    case AlignmentGrabPhase::ApproachingLimit:
      return "APPROACH";
    case AlignmentGrabPhase::SendingGrab:
      return "GRAB";
    case AlignmentGrabPhase::Sent:
      return "SENT";
    case AlignmentGrabPhase::Failed:
      return "FAILED";
  }
  return "WAIT";
}

void TipVisionTestNode::begin_limit_switch_approach()
{
  alignment_grab_phase_ = AlignmentGrabPhase::ApproachingLimit;
  limit_switch_triggered_ = false;
  waiting_for_limit_switch_ = true;
  limit_switch_wait_start_tp_ = std::chrono::steady_clock::now();
  RCLCPP_INFO(
    get_logger(),
    "端头对齐稳定，开始 x 负向前探等待限位: vx=%.3f timeout=%.2fs feedback=%s id=0x%02X",
    compute_approach_vx(), alignment_approach_timeout_s_,
    alignment_limit_switch_feedback_topic_.c_str(),
    static_cast<unsigned int>(alignment_limit_switch_feedback_id_ & 0xFF));
  publish_alignment_command(compute_approach_vx(), 0.0, 0.0, true);
}

void TipVisionTestNode::handle_limit_switch_feedback(const FeedbackMsg::SharedPtr msg)
{
  if (!msg || !waiting_for_limit_switch_.load(std::memory_order_relaxed)) {
    return;
  }
  if (msg->feedback_id != static_cast<uint8_t>(alignment_limit_switch_feedback_id_ & 0xFF)) {
    return;
  }

  limit_switch_triggered_.store(true, std::memory_order_relaxed);
  publish_alignment_stop(true);
  RCLCPP_INFO(
    get_logger(), "收到前方限位触发反馈: seq=%u feedback=0x%02X",
    static_cast<unsigned int>(msg->seq), static_cast<unsigned int>(msg->feedback_id));
}

void TipVisionTestNode::handle_alignment_odom(const OdomMsg::SharedPtr msg)
{
  if (!msg) {
    return;
  }
  std::lock_guard<std::mutex> lock(alignment_odom_mutex_);
  alignment_current_yaw_rad_ = tip_detail::yaw_from_quaternion(msg->pose.pose.orientation);
  alignment_odom_receive_tp_ = std::chrono::steady_clock::now();
  alignment_has_yaw_ = true;
}

bool TipVisionTestNode::read_alignment_yaw(double & yaw_rad, double & age_s)
{
  std::lock_guard<std::mutex> lock(alignment_odom_mutex_);
  if (!alignment_has_yaw_) {
    yaw_rad = 0.0;
    age_s = 0.0;
    return false;
  }
  yaw_rad = alignment_current_yaw_rad_;
  age_s = std::chrono::duration<double>(
    std::chrono::steady_clock::now() - alignment_odom_receive_tp_).count();
  return age_s <= alignment_odom_timeout_s_;
}

rc26_vision::TipHeadingControl TipVisionTestNode::compute_alignment_heading_control(
  bool & stale,
  double & yaw_age_s)
{
  stale = false;
  yaw_age_s = 0.0;
  auto config = make_alignment_config();
  if (!config.heading_hold_enable) {
    return rc26_vision::computeTipHeadingControl(0.0, config);
  }

  double yaw_rad = 0.0;
  if (!read_alignment_yaw(yaw_rad, yaw_age_s)) {
    stale = true;
    rc26_vision::TipHeadingControl control;
    control.aligned = false;
    control.within_gate = false;
    control.allow_lateral = false;
    return control;
  }
  return rc26_vision::computeTipHeadingControl(yaw_rad, config);
}

std::string TipVisionTestNode::update_alignment_control(
  bool has_target,
  int offset_px,
  bool new_inference_result,
  double & out_cmd_vx,
  double & out_cmd_vy,
  bool & out_aligned,
  bool & out_command_published,
  double & out_cmd_wz,
  bool & out_heading_stale,
  bool & out_heading_aligned,
  bool & out_heading_within_gate,
  double & out_heading_error_rad,
  uint8_t & out_grab_seq)
{
  out_cmd_vx = 0.0;
  out_cmd_vy = 0.0;
  out_cmd_wz = 0.0;
  out_aligned = false;
  out_command_published = false;
  out_heading_stale = false;
  out_heading_aligned = true;
  out_heading_within_gate = true;
  out_heading_error_rad = 0.0;
  out_grab_seq = last_grab_seq_;

  if (!alignment_control_enable_) {
    last_grab_state_ = "DISABLED";
    alignment_grab_phase_ = AlignmentGrabPhase::WaitingForAlignment;
    return last_grab_state_;
  }

  if (alignment_grab_phase_ == AlignmentGrabPhase::Failed) {
    publish_alignment_stop(false);
    last_grab_state_ = "FAILED";
    return last_grab_state_;
  }

  if (alignment_grab_phase_ == AlignmentGrabPhase::ApproachingLimit) {
    double yaw_age_s = 0.0;
    const auto heading_control =
      compute_alignment_heading_control(out_heading_stale, yaw_age_s);
    (void)yaw_age_s;
    out_cmd_wz = heading_control.angular_z_radps;
    out_heading_aligned = heading_control.aligned;
    out_heading_within_gate = heading_control.within_gate;
    out_heading_error_rad = heading_control.yaw_error_rad;

    if (limit_switch_triggered_.load(std::memory_order_relaxed)) {
      waiting_for_limit_switch_ = false;
      publish_alignment_stop(true);
      alignment_grab_phase_ = AlignmentGrabPhase::SendingGrab;
    } else if (out_heading_stale) {
      out_command_published = publish_alignment_command(0.0, 0.0, 0.0, true);
      last_grab_state_ = "ODOM_WAIT";
      return last_grab_state_;
    } else if (std::chrono::duration<double>(
                 std::chrono::steady_clock::now() - limit_switch_wait_start_tp_).count() >=
               alignment_approach_timeout_s_) {
      waiting_for_limit_switch_ = false;
      publish_alignment_stop(true);
      alignment_grab_phase_ = AlignmentGrabPhase::Failed;
      last_grab_state_ = "APPROACH_TIMEOUT";
      return last_grab_state_;
    } else {
      out_cmd_vx = heading_control.allow_lateral ? compute_approach_vx() : 0.0;
      out_command_published =
        publish_alignment_command(out_cmd_vx, 0.0, out_cmd_wz, false);
      last_grab_state_ = heading_control.allow_lateral ? "APPROACH" : "HEADING";
      return last_grab_state_;
    }
  }

  if (alignment_grab_phase_ == AlignmentGrabPhase::SendingGrab) {
    publish_alignment_stop(false);
    if (send_grab_tip_command(out_grab_seq)) {
      grab_sent_for_current_target_ = true;
      last_grab_seq_ = out_grab_seq;
      alignment_grab_phase_ = AlignmentGrabPhase::Sent;
      last_grab_state_ = "SENT";
    } else {
      alignment_grab_phase_ = AlignmentGrabPhase::Failed;
      last_grab_state_ = "FAILED";
    }
    return last_grab_state_;
  }

  if (alignment_grab_phase_ == AlignmentGrabPhase::Sent) {
    publish_alignment_stop(false);
    if (!has_target && new_inference_result) {
      ++alignment_lost_count_;
      alignment_stable_count_ = 0;
      if (alignment_lost_count_ >= alignment_lost_stop_frames_) {
        grab_sent_for_current_target_ = false;
        alignment_grab_phase_ = AlignmentGrabPhase::WaitingForAlignment;
      }
    }
    last_grab_state_ = "SENT";
    return last_grab_state_;
  }

  if (!has_target) {
    if (new_inference_result) {
      ++alignment_lost_count_;
      alignment_stable_count_ = 0;
      if (alignment_lost_count_ >= alignment_lost_stop_frames_) {
        grab_sent_for_current_target_ = false;
        alignment_grab_phase_ = AlignmentGrabPhase::WaitingForAlignment;
      }
    }
    out_command_published = publish_alignment_command(0.0, 0.0, 0.0, false);
    last_grab_state_ = alignment_grab_enable_ ? "LOST" : "DISABLED";
    return last_grab_state_;
  }

  double yaw_age_s = 0.0;
  const auto heading_control =
    compute_alignment_heading_control(out_heading_stale, yaw_age_s);
  (void)yaw_age_s;
  out_cmd_wz = heading_control.angular_z_radps;
  out_heading_aligned = heading_control.aligned;
  out_heading_within_gate = heading_control.within_gate;
  out_heading_error_rad = heading_control.yaw_error_rad;
  if (out_heading_stale) {
    alignment_stable_count_ = 0;
    out_command_published = publish_alignment_command(0.0, 0.0, 0.0, true);
    last_grab_state_ = "ODOM_WAIT";
    return last_grab_state_;
  }

  const bool pixel_aligned = std::abs(offset_px) <= alignment_tolerance_px_;
  if (new_inference_result) {
    alignment_lost_count_ = 0;
    out_aligned = pixel_aligned && heading_control.aligned;
    if (out_aligned) {
      ++alignment_stable_count_;
    } else {
      alignment_stable_count_ = 0;
    }
  } else {
    out_aligned = pixel_aligned && heading_control.aligned;
  }

  out_cmd_vy = heading_control.allow_lateral ? compute_alignment_vy(offset_px) : 0.0;
  out_command_published = publish_alignment_command(0.0, out_cmd_vy, out_cmd_wz, false);

  if (!alignment_grab_enable_) {
    last_grab_state_ = "DISABLED";
    return last_grab_state_;
  }
  if (!heading_control.allow_lateral) {
    last_grab_state_ = "HEADING";
    return last_grab_state_;
  }
  if (!out_aligned || alignment_stable_count_ < alignment_stable_frames_) {
    last_grab_state_ = "WAIT";
    return last_grab_state_;
  }
  if (alignment_grab_once_per_target_ && grab_sent_for_current_target_) {
    last_grab_state_ = "SENT";
    return last_grab_state_;
  }

  const auto now = std::chrono::steady_clock::now();
  if (last_grab_attempt_tp_ != std::chrono::steady_clock::time_point{} &&
    std::chrono::duration<double>(now - last_grab_attempt_tp_).count() < alignment_grab_cooldown_s_)
  {
    last_grab_state_ = "COOLDOWN";
    return last_grab_state_;
  }

  last_grab_attempt_tp_ = now;
  begin_limit_switch_approach();
  last_grab_state_ = "APPROACH";
  return last_grab_state_;
}

bool TipVisionTestNode::send_grab_tip_command(uint8_t & out_seq)
{
  out_seq = 0U;
  if (!grab_command_client_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "GRAB_TIP 发送跳过: 服务客户端未创建。");
    return false;
  }
  if (!grab_command_client_->service_is_ready()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "GRAB_TIP 发送跳过: 服务 %s 未就绪。",
      alignment_grab_service_name_.c_str());
    return false;
  }

  auto request = std::make_shared<SendCommandSrv::Request>();
  request->command_id = static_cast<uint8_t>(alignment_grab_command_id_);
  request->payload = alignment_grab_payload_;

  const uint64_t token = grab_request_generation_.fetch_add(1, std::memory_order_relaxed) + 1U;
  {
    std::lock_guard<std::mutex> lock(grab_response_mutex_);
    grab_response_seen_ = false;
    grab_response_accepted_ = false;
    grab_response_seq_ = 0U;
  }

  try {
    grab_command_client_->async_send_request(
      request,
      [this, token](rclcpp::Client<SendCommandSrv>::SharedFuture future) {
        if (token != grab_request_generation_.load(std::memory_order_relaxed)) {
          return;
        }
        bool accepted = false;
        uint8_t seq = 0U;
        try {
          const auto response = future.get();
          accepted = response && response->accepted;
          if (response) {
            seq = response->seq;
          }
        } catch (const std::exception & ex) {
          RCLCPP_WARN(get_logger(), "GRAB_TIP 响应异常: %s", ex.what());
        }

        {
          std::lock_guard<std::mutex> lock(grab_response_mutex_);
          grab_response_accepted_ = accepted;
          grab_response_seq_ = seq;
          grab_response_seen_ = true;
        }
        grab_response_cv_.notify_one();
      });
  } catch (const std::exception & ex) {
    RCLCPP_WARN(get_logger(), "GRAB_TIP 发送异常: %s", ex.what());
    return false;
  }

  std::unique_lock<std::mutex> lock(grab_response_mutex_);
  const bool response_seen = grab_response_cv_.wait_for(
    lock, std::chrono::milliseconds(alignment_grab_service_timeout_ms_),
    [this]() { return grab_response_seen_; });

  if (!response_seen) {
    grab_request_generation_.fetch_add(1, std::memory_order_relaxed);
    RCLCPP_WARN(
      get_logger(), "GRAB_TIP 发送超时: service=%s timeout_ms=%d",
      alignment_grab_service_name_.c_str(), alignment_grab_service_timeout_ms_);
    return false;
  }

  if (!grab_response_accepted_) {
    RCLCPP_WARN(
      get_logger(), "GRAB_TIP 发送被拒绝: cmd=%s payload=%s",
      tip_detail::byte_to_hex(request->command_id).c_str(),
      tip_detail::bytes_to_hex(request->payload).c_str());
    return false;
  }

  out_seq = grab_response_seq_;
  RCLCPP_INFO(
    get_logger(), "GRAB_TIP 已发送: cmd=%s payload=%s seq=%u",
    tip_detail::byte_to_hex(request->command_id).c_str(),
    tip_detail::bytes_to_hex(request->payload).c_str(),
    static_cast<unsigned int>(out_seq));
  return true;
}

}  // namespace rc26_vision::test

// ---- tip_vision_test_node_camera.cpp ----

#include <algorithm>
#include <filesystem>
#include <limits>
#include <regex>

namespace rc26_vision::test {

void TipVisionTestNode::configure_camera_properties()
{
  camera_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
  camera_.set(cv::CAP_PROP_FRAME_WIDTH, static_cast<double>(camera_width_));
  camera_.set(cv::CAP_PROP_FRAME_HEIGHT, static_cast<double>(camera_height_));
  camera_.set(cv::CAP_PROP_FPS, static_cast<double>(camera_fps_));

  const int actual_w = static_cast<int>(camera_.get(cv::CAP_PROP_FRAME_WIDTH));
  const int actual_h = static_cast<int>(camera_.get(cv::CAP_PROP_FRAME_HEIGHT));
  if (actual_w != camera_width_ || actual_h != camera_height_) {
    camera_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('Y', 'U', 'Y', 'V'));
    camera_.set(cv::CAP_PROP_FRAME_WIDTH, static_cast<double>(camera_width_));
    camera_.set(cv::CAP_PROP_FRAME_HEIGHT, static_cast<double>(camera_height_));
    camera_.set(cv::CAP_PROP_FPS, static_cast<double>(camera_fps_));
  }
}

bool TipVisionTestNode::open_camera_by_index(int index, std::string & opened_source)
{
  if (camera_.isOpened()) {
    camera_.release();
  }
  bool opened = camera_.open(index, cv::CAP_V4L2);
  if (!opened) {
    opened = camera_.open(index);
  }
  if (!opened) {
    return false;
  }

  configure_camera_properties();
  cv::Mat test_frame;
  if (!camera_.read(test_frame) || test_frame.empty()) {
    RCLCPP_WARN(
      get_logger(),
      "相机索引 %d 已打开但读取首帧失败；释放此候选。",
      index);
    camera_.release();
    return false;
  }

  opened_source = "index:" + std::to_string(index);
  return true;
}

bool TipVisionTestNode::open_camera_by_path(const std::string & path, std::string & opened_source)
{
  if (camera_.isOpened()) {
    camera_.release();
  }
  bool opened = camera_.open(path, cv::CAP_V4L2);
  if (!opened) {
    opened = camera_.open(path);
  }
  if (!opened) {
    return false;
  }

  configure_camera_properties();
  cv::Mat test_frame;
  if (!camera_.read(test_frame) || test_frame.empty()) {
    RCLCPP_WARN(
      get_logger(),
      "相机路径 '%s' 已打开但读取首帧失败；释放此候选。",
      path.c_str());
    camera_.release();
    return false;
  }

  opened_source = path;
  return true;
}

std::vector<std::string> TipVisionTestNode::discover_video_devices() const
{
  std::vector<std::string> devices;
  const std::filesystem::path dev_path("/dev");
  if (!std::filesystem::exists(dev_path)) {
    return devices;
  }

  std::regex video_regex("^video[0-9]+$");
  for (const auto & entry : std::filesystem::directory_iterator(dev_path)) {
    if (!entry.is_character_file()) {
      continue;
    }
    const std::string filename = entry.path().filename().string();
    if (std::regex_match(filename, video_regex)) {
      devices.push_back(entry.path().string());
    }
  }

  std::sort(
    devices.begin(), devices.end(),
    [](const std::string & lhs, const std::string & rhs) {
      auto parse_id = [](const std::string & value) -> int {
          const std::size_t pos = value.find("video");
          if (pos == std::string::npos) {
            return std::numeric_limits<int>::max();
          }
          try {
            return std::stoi(value.substr(pos + 5U));
          } catch (...) {
            return std::numeric_limits<int>::max();
          }
        };
      return parse_id(lhs) < parse_id(rhs);
    });

  return devices;
}

int TipVisionTestNode::parse_video_index_from_path(const std::string & path) const
{
  static const std::regex re(".*/video([0-9]+)$");
  std::smatch match;
  if (!std::regex_match(path, match, re) || match.size() < 2U) {
    return -1;
  }
  try {
    return std::stoi(match[1].str());
  } catch (...) {
    return -1;
  }
}

bool TipVisionTestNode::init_camera()
{
  std::string opened_source;
  bool opened = false;

  if (!camera_device_.empty()) {
    RCLCPP_INFO(get_logger(), "正在尝试首选相机路径 '%s'...", camera_device_.c_str());
    opened = open_camera_by_path(camera_device_, opened_source);
  } else {
    RCLCPP_INFO(get_logger(), "正在尝试首选相机索引 %d...", camera_index_);
    opened = open_camera_by_index(camera_index_, opened_source);
  }

  if (!opened && auto_scan_camera_) {
    RCLCPP_WARN(
      get_logger(),
      "首选相机未能产生帧；auto_scan_camera=true，正在扫描其他 /dev/video* 设备。");
    const std::vector<std::string> candidates = discover_video_devices();
    for (const auto & candidate : candidates) {
      if (!camera_device_.empty() && candidate == camera_device_) {
        continue;
      }
      const int candidate_index = parse_video_index_from_path(candidate);
      RCLCPP_INFO(get_logger(), "正在尝试备选相机 %s...", candidate.c_str());
      if (candidate_index >= 0 && open_camera_by_index(candidate_index, opened_source)) {
        opened = true;
        break;
      }
      if (open_camera_by_path(candidate, opened_source)) {
        opened = true;
        break;
      }
    }
  }

  if (!opened) {
    RCLCPP_ERROR(
      get_logger(), "无法打开可用的相机 (camera_index=%d camera_device='%s')。",
      camera_index_, camera_device_.c_str());
    return false;
  }

  const int actual_w = static_cast<int>(camera_.get(cv::CAP_PROP_FRAME_WIDTH));
  const int actual_h = static_cast<int>(camera_.get(cv::CAP_PROP_FRAME_HEIGHT));
  const int actual_fps = static_cast<int>(camera_.get(cv::CAP_PROP_FPS));
  selected_camera_source_ = opened_source;

  RCLCPP_INFO(
    get_logger(), "相机已打开 %s (后端=%s): 实际=%dx%d@%dfps 请求=%dx%d@%d",
    selected_camera_source_.c_str(), camera_.getBackendName().c_str(), actual_w, actual_h, actual_fps,
    camera_width_, camera_height_, camera_fps_);
  return true;
}

}  // namespace rc26_vision::test

// ---- tip_vision_test_node_inference.cpp ----

#include "rc26_vision/inference/config/model_profile_loader.hpp"
#include "rc26_vision/inference/runtime/engine_factory.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <unordered_set>

namespace rc26_vision::test {

bool TipVisionTestNode::run_inference_on_frame(
  const cv::Mat & frame_bgr, uint64_t source_frame_seq,
  InferenceResult & result)
{
  if (!engine_) {
    return false;
  }

  const auto infer_start = std::chrono::steady_clock::now();
  std::vector<rc26_vision::Detection> detections = engine_->infer(frame_bgr);
  const auto infer_end = std::chrono::steady_clock::now();

  std::sort(
    detections.begin(), detections.end(),
    [](const rc26_vision::Detection & lhs, const rc26_vision::Detection & rhs) {
      return lhs.score > rhs.score;
    });

  if (single_target_mode_ && !detections.empty()) {
    detections.resize(1U);
  } else if (max_categories_ > 0 && model_profile_.labels.size() > 1U && !detections.empty()) {
    std::vector<rc26_vision::Detection> filtered;
    filtered.reserve(static_cast<std::size_t>(max_categories_));
    std::unordered_set<int> seen_classes;
    seen_classes.reserve(static_cast<std::size_t>(max_categories_) * 2U);

    for (const auto & det : detections) {
      if (seen_classes.find(det.class_id) != seen_classes.end()) {
        continue;
      }
      filtered.push_back(det);
      seen_classes.insert(det.class_id);
      if (static_cast<int>(filtered.size()) >= max_categories_) {
        break;
      }
    }
    detections.swap(filtered);
  }

  InferenceResult local_result;
  local_result.valid = true;
  local_result.inference_seq = ++inference_result_seq_;
  local_result.source_frame_seq = source_frame_seq;
  local_result.detection_count = detections.size();
  local_result.infer_ms =
    std::chrono::duration<double, std::milli>(infer_end - infer_start).count();
  local_result.updated_tp = std::chrono::steady_clock::now();

  if (show_window_) {
    local_result.detections = detections;
  }

  const auto selection = rc26_vision::updateTipAlignmentTarget(
    detections, frame_bgr.cols, target_class_ids_, alignment_target_lock_state_,
    make_alignment_config());
  local_result.target_locked = selection.locked;
  local_result.target_lock_lost_count = selection.lock_lost_count;
  if (selection.has_target) {
    local_result.primary_target = selection.target;
  }
  local_result.has_target = local_result.primary_target.has_value();
  if (local_result.has_target) {
    local_result.box_w = local_result.primary_target->box.width;
    local_result.box_cx =
      local_result.primary_target->box.x + local_result.primary_target->box.width / 2;
    local_result.offset_px = local_result.box_cx - frame_bgr.cols / 2;
    local_result.class_id = local_result.primary_target->class_id;
  }

  result = std::move(local_result);
  return true;
}

void TipVisionTestNode::submit_async_frame(const cv::Mat & frame_bgr, uint64_t source_frame_seq)
{
  {
    std::lock_guard<std::mutex> lock(async_mutex_);
    async_pending_frame_ = frame_bgr.clone();
    async_pending_frame_seq_ = source_frame_seq;
    async_has_pending_frame_ = true;
  }
  async_cv_.notify_one();
}

bool TipVisionTestNode::copy_latest_inference_result(InferenceResult & result)
{
  std::lock_guard<std::mutex> lock(inference_result_mutex_);
  if (!has_inference_result_) {
    return false;
  }
  result = latest_inference_result_;
  return true;
}

void TipVisionTestNode::async_inference_worker_loop()
{
  uint64_t consumed_frame_seq = 0U;
  while (rclcpp::ok()) {
    cv::Mat frame_bgr;
    uint64_t source_frame_seq = 0U;
    {
      std::unique_lock<std::mutex> lock(async_mutex_);
      async_cv_.wait(lock, [&]() {
        return async_stop_requested_ ||
               (async_has_pending_frame_ && async_pending_frame_seq_ != consumed_frame_seq);
      });
      if (async_stop_requested_) {
        break;
      }
      frame_bgr = std::move(async_pending_frame_);
      source_frame_seq = async_pending_frame_seq_;
      consumed_frame_seq = source_frame_seq;
      async_has_pending_frame_ = false;
    }

    InferenceResult result;
    if (!run_inference_on_frame(frame_bgr, source_frame_seq, result)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "共享引擎推理失败。");
      continue;
    }

    {
      std::lock_guard<std::mutex> lock(inference_result_mutex_);
      latest_inference_result_ = std::move(result);
      has_inference_result_ = true;
    }
  }
}

bool TipVisionTestNode::start_async_inference_worker()
{
  async_stop_requested_ = false;
  has_inference_result_ = false;
  inference_result_seq_ = 0U;
  try {
    async_worker_thread_ = std::thread(&TipVisionTestNode::async_inference_worker_loop, this);
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(get_logger(), "启动异步推理工作线程失败: %s", ex.what());
    return false;
  }
  return true;
}

void TipVisionTestNode::stop_async_inference_worker()
{
  {
    std::lock_guard<std::mutex> lock(async_mutex_);
    async_stop_requested_ = true;
  }
  async_cv_.notify_all();
  if (async_worker_thread_.joinable()) {
    async_worker_thread_.join();
  }
}

bool TipVisionTestNode::initialize()
{
  if (!init_inference()) {
    return false;
  }
  if (!resolve_target_class_ids()) {
    return false;
  }

  RCLCPP_INFO(
    get_logger(),
    "检测策略: model_id=%s 单目标模式=%s 最大类别数=%d 使用预测标签=%s",
    model_id_.c_str(), single_target_mode_ ? "true" : "false", max_categories_,
    use_predicted_label_ ? "true" : "false");

  if (!init_camera()) {
    return false;
  }

  alignment_stable_count_ = 0;
  alignment_lost_count_ = 0;
  alignment_grab_phase_ = AlignmentGrabPhase::WaitingForAlignment;
  grab_sent_for_current_target_ = false;
  last_grab_state_ = alignment_grab_enable_ ? "WAIT" : "DISABLED";
  last_grab_seq_ = 0U;
  last_alignment_command_tp_ = std::chrono::steady_clock::time_point{};
  last_grab_attempt_tp_ = std::chrono::steady_clock::time_point{};
  limit_switch_wait_start_tp_ = std::chrono::steady_clock::time_point{};
  alignment_zero_published_ = false;
  waiting_for_limit_switch_ = false;
  limit_switch_triggered_ = false;
  {
    std::lock_guard<std::mutex> lock(alignment_odom_mutex_);
    alignment_has_yaw_ = false;
    alignment_current_yaw_rad_ = 0.0;
    alignment_odom_receive_tp_ = std::chrono::steady_clock::time_point{};
  }
  grab_request_generation_.fetch_add(1, std::memory_order_relaxed);
  {
    std::lock_guard<std::mutex> lock(grab_response_mutex_);
    grab_response_seen_ = false;
    grab_response_accepted_ = false;
    grab_response_seq_ = 0U;
  }
  alignment_target_lock_state_.reset();

  create_alignment_interfaces();

  return true;
}

bool TipVisionTestNode::init_inference()
{
  try {
    const auto config = rc26_vision::ProfileLoader::loadFromYaml(vision_config_file_);
    rc26_vision::ProfileLoader::validate(config);
    const auto profile_it = config.profiles.find(model_id_);
    if (profile_it == config.profiles.end()) {
      RCLCPP_ERROR(
        get_logger(), "视觉 model_id '%s' 在 %s 中未找到",
        model_id_.c_str(), vision_config_file_.c_str());
      return false;
    }

    model_profile_ = profile_it->second;
    class_names_ = model_profile_.labels;
    engine_ = rc26_vision::createInferenceEngine(model_profile_);
    RCLCPP_INFO(
      get_logger(), "端头推理使用共享引擎配置 '%s': 模型=%s 标签数=%zu",
      model_profile_.id.c_str(), model_profile_.model_path.c_str(), class_names_.size());
    return true;
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(get_logger(), "初始化共享推理引擎失败: %s", ex.what());
    return false;
  }
}

}  // namespace rc26_vision::test

// ---- tip_vision_test_node_overlay.cpp ----

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>

namespace rc26_vision::test {

void TipVisionTestNode::draw_alignment_guides(
  cv::Mat & frame_bgr,
  bool has_target,
  int box_cx,
  bool aligned) const
{
  if (!alignment_draw_guides_ || frame_bgr.empty()) {
    return;
  }

  const int center_x = frame_bgr.cols / 2;
  const cv::Scalar target_line_color = aligned ? cv::Scalar(70, 220, 70) : cv::Scalar(0, 210, 255);
  const cv::Scalar box_line_color = aligned ? cv::Scalar(70, 220, 70) : cv::Scalar(80, 120, 255);

  cv::line(
    frame_bgr, cv::Point(center_x, 0), cv::Point(center_x, frame_bgr.rows - 1),
    target_line_color, 2, cv::LINE_AA);
  if (alignment_tolerance_px_ > 0) {
    const int left_tol = std::clamp(center_x - alignment_tolerance_px_, 0, frame_bgr.cols - 1);
    const int right_tol = std::clamp(center_x + alignment_tolerance_px_, 0, frame_bgr.cols - 1);
    cv::line(
      frame_bgr, cv::Point(left_tol, 0), cv::Point(left_tol, frame_bgr.rows - 1),
      cv::Scalar(90, 90, 90), 1, cv::LINE_AA);
    cv::line(
      frame_bgr, cv::Point(right_tol, 0), cv::Point(right_tol, frame_bgr.rows - 1),
      cv::Scalar(90, 90, 90), 1, cv::LINE_AA);
  }

  if (has_target && box_cx >= 0 && box_cx < frame_bgr.cols) {
    cv::line(
      frame_bgr, cv::Point(box_cx, 0), cv::Point(box_cx, frame_bgr.rows - 1),
      box_line_color, 2, cv::LINE_AA);
  }
}

void TipVisionTestNode::draw_alignment_overlay(
  cv::Mat & frame_bgr,
  const AlignmentOverlayInfo & info) const
{
  std::vector<std::string> lines;
  lines.reserve(7U);

  const std::string align_state =
    !info.control_enabled ? "OFF" :
    (!info.has_target ? "LOST" : (info.aligned ? "OK" : "MOVE"));

  {
    std::ostringstream ss;
    ss << "ALIGN " << (info.control_enabled ? "ON" : "OFF")
       << " target=" << (info.has_target ? "YES" : "NO")
       << " state=" << align_state
       << " label=" << info.label;
    lines.push_back(ss.str());
  }

  {
    std::ostringstream ss;
    ss << "CTRL off=" << info.offset_px
       << " tol=" << info.tolerance_px
       << " vx=" << std::fixed << std::setprecision(3) << info.cmd_vx
       << " vy=" << std::setprecision(3) << info.cmd_vy
       << " wz=" << std::setprecision(3) << info.cmd_wz
       << " sent=" << (info.command_published ? "YES" : "RATE");
    lines.push_back(ss.str());
  }

  {
    std::ostringstream ss;
    ss << "HEAD " << (info.heading_enabled ? "ON" : "OFF")
       << " stale=" << (info.heading_stale ? "YES" : "NO")
       << " err=" << std::fixed << std::setprecision(3) << info.heading_error_rad
       << " ok=" << (info.heading_aligned ? "YES" : "NO")
       << " gate=" << (info.heading_within_gate ? "YES" : "NO");
    lines.push_back(ss.str());
  }

  {
    std::ostringstream ss;
    ss << "LOCK " << (info.target_locked ? "ON" : "OFF")
       << " miss=" << info.target_lock_lost_count;
    lines.push_back(ss.str());
  }

  {
    std::ostringstream ss;
    ss << "STABLE " << info.stable_count << "/" << info.stable_required
       << " lost=" << info.lost_count;
    lines.push_back(ss.str());
  }

  {
    std::ostringstream ss;
    ss << "LIMIT " << (info.grab_enabled ? "ON" : "OFF")
       << " trig=" << (info.limit_switch_triggered ? "YES" : "NO")
       << " phase=" << info.grab_phase;
    lines.push_back(ss.str());
  }

  {
    std::ostringstream ss;
    ss << "GRAB " << (info.grab_enabled ? info.grab_state : "DISABLED")
       << " cmd=" << tip_detail::byte_to_hex(info.grab_command_id)
       << " seq=" << static_cast<unsigned int>(info.grab_seq);
    lines.push_back(ss.str());
  }

  constexpr int kPanelX = 10;
  constexpr int kPanelY = 40;
  constexpr int kPanelPadding = 8;
  constexpr int kLineGap = 6;
  constexpr double kFontScale = 0.52;
  constexpr int kThickness = 1;

  int max_text_width = 0;
  int total_text_height = 0;
  std::vector<cv::Size> text_sizes;
  text_sizes.reserve(lines.size());
  for (const auto & line : lines) {
    int baseline = 0;
    const cv::Size text_size =
      cv::getTextSize(line, cv::FONT_HERSHEY_SIMPLEX, kFontScale, kThickness, &baseline);
    text_sizes.push_back(text_size);
    max_text_width = std::max(max_text_width, text_size.width);
    total_text_height += text_size.height;
  }

  const int panel_width = std::min(
    frame_bgr.cols - kPanelX * 2,
    std::max(120, max_text_width + kPanelPadding * 2));
  const int panel_height = std::min(
    frame_bgr.rows - kPanelY - 10,
    total_text_height + kPanelPadding * 2 + kLineGap * static_cast<int>(lines.size() - 1U));
  if (panel_width <= 0 || panel_height <= 0) {
    return;
  }

  const cv::Rect panel_rect(kPanelX, kPanelY, panel_width, panel_height);
  const cv::Scalar panel_color =
    !info.control_enabled ? cv::Scalar(55, 55, 55) :
    (info.aligned ? cv::Scalar(35, 70, 35) : cv::Scalar(40, 45, 75));
  cv::rectangle(frame_bgr, panel_rect, panel_color, cv::FILLED);
  cv::rectangle(frame_bgr, panel_rect, cv::Scalar(180, 180, 180), 1, cv::LINE_AA);

  int text_y = panel_rect.y + kPanelPadding;
  for (std::size_t i = 0; i < lines.size(); ++i) {
    text_y += text_sizes[i].height;
    cv::putText(
      frame_bgr, lines[i], cv::Point(panel_rect.x + kPanelPadding, text_y),
      cv::FONT_HERSHEY_SIMPLEX, kFontScale, cv::Scalar(255, 255, 255), kThickness,
      cv::LINE_AA);
    text_y += kLineGap;
  }
}

void TipVisionTestNode::draw_detections(
  cv::Mat & frame_bgr, const std::vector<rc26_vision::Detection> & detections) const
{
  for (const auto & det : detections) {
    const int x1 = static_cast<int>(std::floor(det.x1));
    const int y1 = static_cast<int>(std::floor(det.y1));
    const int x2 = static_cast<int>(std::ceil(det.x2));
    const int y2 = static_cast<int>(std::ceil(det.y2));
    const cv::Rect box(x1, y1, std::max(0, x2 - x1), std::max(0, y2 - y1));
    if (box.width <= 0 || box.height <= 0) {
      continue;
    }

    cv::rectangle(frame_bgr, box, cv::Scalar(0, 255, 0), 2, cv::LINE_AA);

    std::ostringstream ss;
    std::string label = target_name_.empty() ? "JK" : target_name_;
    if (use_predicted_label_) {
      if (det.class_id >= 0 && det.class_id < static_cast<int>(class_names_.size())) {
        label = class_names_[static_cast<std::size_t>(det.class_id)];
      } else if (det.class_id >= 0) {
        label = "class_" + std::to_string(det.class_id);
      }
    }

    ss << label;
    if (use_predicted_label_) {
      ss << "[" << det.class_id << "]";
    }
    ss << " " << std::fixed << std::setprecision(2) << det.score;
    const std::string text = ss.str();

    int baseline = 0;
    const cv::Size text_size =
      cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, 0.6, 2, &baseline);
    int text_y = std::max(0, box.y - 8);
    int text_x = std::max(0, box.x);

    const cv::Rect bg_rect(
      text_x,
      std::max(0, text_y - text_size.height - 4),
      std::min(frame_bgr.cols - text_x, text_size.width + 8),
      text_size.height + 8);
    cv::rectangle(frame_bgr, bg_rect, cv::Scalar(0, 120, 0), cv::FILLED);
    cv::putText(
      frame_bgr, text,
      cv::Point(text_x + 4, bg_rect.y + bg_rect.height - 5),
      cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 2, cv::LINE_AA);
  }
}

}  // namespace rc26_vision::test

// ---- tip_vision_test_node_main.cpp ----

#include <cstdio>
#include <cstdlib>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  int rc = 0;
  try {
    auto * node = new rc26_vision::test::TipVisionTestNode();
    rc = node->run();
  } catch (const std::exception & ex) {
    std::fprintf(stderr, "致命错误: %s\n", ex.what());
    rc = 2;
  } catch (...) {
    std::fprintf(stderr, "未知致命错误\n");
    rc = 3;
  }

  std::fflush(nullptr);
  // ROS logging and Aidlite teardown are unstable on this target during process
  // shutdown. Exit the process directly after the run loop finishes.
  std::_Exit(rc);
}
