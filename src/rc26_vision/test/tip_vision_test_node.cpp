// Combined tip vision test node implementation. This file is intentionally
// kept under package-root test/ so the test-only node does not publish
// private headers through include/.

#include <rclcpp/rclcpp.hpp>

#include <opencv2/opencv.hpp>

#include <array>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "rc26_serial/serial_driver.hpp"
#include "rc26_vision/engines/inference_engine.hpp"
#include "rc26_vision/runtime/model_profile.hpp"

namespace rc26_vision::test {

constexpr double kMainPyRefWidthRatio = 0.80;
constexpr rc26_serial::CommandID kTipVisionCommand =
  rc26_serial::CommandID::TIP_VISION;

class TipVisionTestNode : public rclcpp::Node
{
public:
  TipVisionTestNode();
  ~TipVisionTestNode() override;

  int run();

private:
  struct OffsetDecision
  {
    uint8_t dir_code{0x00};
    uint8_t amp_code{0x00};
  };

  struct SerialOverlayInfo
  {
    bool serial_enabled{false};
    bool serial_open{false};
    bool tx_sent{false};
    bool has_target{false};
    uint16_t ts16{0x0000};
    uint8_t packet_size{0x00};
    uint8_t command_id{static_cast<uint8_t>(kTipVisionCommand)};
    uint8_t seq{0x00};
    std::string label{"LOST"};
    int offset_px{0};
    uint8_t dir_code{0x00};
    uint8_t amp_code{0x00};
    bool grab_ready{false};
    int stable_ok_count{0};
    std::string packet_hex{"-"};
  };

  struct TargetCandidate
  {
    cv::Rect box{};
    int class_id{-1};
    float score{0.0F};
  };

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
    uint8_t dir_code{0x00};
    uint8_t amp_code{0x03};
    bool grab_ready_now{false};
    std::size_t detection_count{0U};
    double infer_ms{0.0};
    std::chrono::steady_clock::time_point updated_tp{};
  };

  void declare_parameters();
  void load_parameters();
  std::string resolve_resource_path(
    const std::string & configured_path,
    const std::filesystem::path & package_relative_default) const;
  bool resolve_target_class_ids();
  bool is_target_class(int class_id) const;
  std::string class_id_to_label(int class_id) const;
  std::optional<TargetCandidate> select_primary_target(
    const std::vector<rc26_vision::Detection> & detections) const;
  OffsetDecision classify_offset(int offset_px, int ref_w) const;

  bool is_serial_open() const;
  void close_serial_driver();
  bool open_serial_port();
  void serial_reconnect_if_needed();
  bool write_serial_packet(const std::array<uint8_t, 5> & packet, uint8_t & out_seq);

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
  void draw_serial_overlay(cv::Mat & frame_bgr, const SerialOverlayInfo & info) const;
  void draw_detections(
    cv::Mat & frame_bgr, const std::vector<rc26_vision::Detection> & detections) const;

  std::string vision_config_file_;
  std::string model_id_{"tip_test"};
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
  bool serial_enable_{true};
  std::string serial_device_{"/dev/ttyUSB1"};
  int serial_baud_{115200};
  int serial_data_bits_{8};
  int serial_stop_bits_{1};
  std::string serial_parity_{"none"};
  std::vector<std::string> target_labels_{"D_0", "D_1"};
  std::vector<int> target_class_ids_;
  double grab_min_width_ratio_{0.06};
  int ok_stable_frames_{3};
  double center_ratio_{0.08};
  double small_ratio_{0.20};
  double medium_ratio_{0.35};

  rc26_vision::InferenceEnginePtr engine_;
  cv::VideoCapture camera_;

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
  int stable_ok_count_{0};
  std::shared_ptr<rc26_decision::SerialDriver> serial_driver_;
  std::chrono::steady_clock::time_point last_serial_open_attempt_tp_{};
  std::chrono::milliseconds serial_reconnect_interval_{1000};

  uint64_t frames_since_log_{0};
  std::chrono::steady_clock::time_point last_log_tp_{};
};

}  // namespace rc26_vision::test

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cctype>
#include <cstdint>
#include <filesystem>
#include <iomanip>
#include <sstream>
#include <string>

namespace rc26_vision::test::tip_detail {

constexpr double kPi = 3.14159265358979323846;

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

inline std::string to_lower_copy(std::string value)
{
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  return value;
}

inline bool ends_with_ignore_case(const std::string & value, const std::string & suffix)
{
  if (value.size() < suffix.size()) {
    return false;
  }
  return value.compare(value.size() - suffix.size(), suffix.size(), suffix) == 0;
}

inline std::filesystem::path get_rc26_vision_share_dir()
{
  try {
    return ament_index_cpp::get_package_share_directory("rc26_vision");
  } catch (...) {
    return {};
  }
}

inline uint16_t monotonic_tick_ms16()
{
  const auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
  return static_cast<uint16_t>(now_ms & 0xFFFF);
}

inline std::array<uint8_t, 5> build_tip_payload(
  bool grab_ready, uint8_t dir_code, uint8_t amp_code, uint16_t ts16)
{
  return {
    static_cast<uint8_t>(grab_ready ? 0x01U : 0x00U),
    dir_code,
    amp_code,
    static_cast<uint8_t>(ts16 & 0xFFU),
    static_cast<uint8_t>((ts16 >> 8U) & 0xFFU)
  };
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
  stop_async_inference_worker();
  close_serial_driver();

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
  double current_infer_ms = 0.0;
  double display_camera_fps = 0.0;
  double display_infer_fps = 0.0;
  uint8_t current_dir_code = 0x00;
  uint8_t current_amp_code = 0x03;
  uint8_t last_dir_code = 0x00;

  while (rclcpp::ok()) {
    cv::Mat frame_bgr;
    if (!camera_.read(frame_bgr) || frame_bgr.empty()) {
      const auto now = std::chrono::steady_clock::now();
      if (now - last_capture_warn_tp > 2s) {
        RCLCPP_WARN(get_logger(), "Camera frame grab failed, retrying...");
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
          get_logger(), *get_clock(), 2000, "Inference failed through shared engine.");
        continue;
      }
      latest_inference_result_ = sync_result;
      has_inference_result_ = true;
    }

    if (copy_latest_inference_result(latest_result)) {
      has_latest_result = true;
      current_detection_count = latest_result.detection_count;
      current_infer_ms = latest_result.infer_ms;

      if (latest_result.inference_seq != last_applied_inference_seq) {
        last_applied_inference_seq = latest_result.inference_seq;
        current_has_target = latest_result.has_target;
        if (latest_result.has_target) {
          current_box_cx = latest_result.box_cx;
          current_offset_px = latest_result.offset_px;
          current_class_id = latest_result.class_id;
          current_dir_code = latest_result.dir_code;
          current_amp_code = latest_result.amp_code;
          last_dir_code = current_dir_code;
          if (latest_result.grab_ready_now) {
            ++stable_ok_count_;
          } else {
            stable_ok_count_ = 0;
          }
        } else {
          current_box_cx = -1;
          current_offset_px = 0;
          current_class_id = -1;
          current_dir_code = last_dir_code;
          current_amp_code = 0x03;
          stable_ok_count_ = 0;
        }
      }
    }

    if (show_window_ && has_latest_result) {
      draw_detections(frame_bgr, latest_result.detections);
    }

    const bool grab_ready = (stable_ok_count_ >= ok_stable_frames_);
    const uint16_t tx_ts16 = tip_detail::monotonic_tick_ms16();
    const std::array<uint8_t, 5> serial_packet =
      tip_detail::build_tip_payload(grab_ready, current_dir_code, current_amp_code, tx_ts16);

    bool serial_tx_sent = false;
    uint8_t serial_tx_seq = 0U;

    if (serial_enable_) {
      serial_tx_sent = write_serial_packet(serial_packet, serial_tx_seq);
      if (serial_tx_sent) {
        const std::string label = class_id_to_label(current_class_id);
        RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 500,
          "TX cmd=0x%02X seq=%u payload=%s | label=%s id=%d cx=%d offset=%d dir=0x%02X amp=0x%02X ts=0x%04X "
          "grab=%s stable=%d/%d target=%s",
          static_cast<unsigned int>(kTipVisionCommand),
          static_cast<unsigned int>(serial_tx_seq),
          tip_detail::bytes_to_hex(serial_packet).c_str(),
          label.c_str(), current_class_id, current_box_cx, current_offset_px,
          static_cast<unsigned int>(current_dir_code), static_cast<unsigned int>(current_amp_code),
          static_cast<unsigned int>(tx_ts16),
          grab_ready ? "OK" : "WAIT", stable_ok_count_, ok_stable_frames_,
          current_has_target ? "yes" : "no");
      }
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
      SerialOverlayInfo serial_info;
      serial_info.serial_enabled = serial_enable_;
      serial_info.serial_open = is_serial_open();
      serial_info.tx_sent = serial_tx_sent;
      serial_info.has_target = current_has_target;
      serial_info.ts16 = tx_ts16;
      serial_info.packet_size = static_cast<uint8_t>(serial_packet.size());
      serial_info.command_id = static_cast<uint8_t>(kTipVisionCommand);
      serial_info.seq = serial_tx_seq;
      serial_info.label = class_id_to_label(current_class_id);
      serial_info.offset_px = current_offset_px;
      serial_info.dir_code = current_dir_code;
      serial_info.amp_code = current_amp_code;
      serial_info.grab_ready = grab_ready;
      serial_info.stable_ok_count = stable_ok_count_;
      serial_info.packet_hex = tip_detail::bytes_to_hex(serial_packet);
      draw_serial_overlay(frame_bgr, serial_info);

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
        RCLCPP_INFO(get_logger(), "Exit requested by keyboard.");
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
        get_logger(), "loop_fps=%.2f infer_fps=%.2f infer_ms=%.2f detections=%zu",
        loop_fps, infer_fps, current_infer_ms, current_detection_count);
      frames_since_log_ = 0;
      last_log_tp_ = now;
      last_logged_inference_seq = current_inference_seq;
    }
  }

  stop_async_inference_worker();
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
  this->declare_parameter<std::string>("model_id", "tip_test");
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
  this->declare_parameter<bool>("serial_enable", true);
  this->declare_parameter<std::string>("serial_device", "/dev/ttyUSB1");
  this->declare_parameter<int>("serial_baud", 115200);
  this->declare_parameter<int>("serial_data_bits", 8);
  this->declare_parameter<int>("serial_stop_bits", 1);
  this->declare_parameter<std::string>("serial_parity", "none");
  this->declare_parameter<std::vector<std::string>>(
    "target_labels", std::vector<std::string>{"JK"});
  this->declare_parameter<double>("grab_min_width_ratio", 0.06);
  this->declare_parameter<int>("ok_stable_frames", 3);
  this->declare_parameter<double>("center_ratio", 0.08);
  this->declare_parameter<double>("small_ratio", 0.20);
  this->declare_parameter<double>("medium_ratio", 0.35);
}

void TipVisionTestNode::load_parameters()
{
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
  serial_enable_ = this->get_parameter("serial_enable").as_bool();
  serial_device_ = this->get_parameter("serial_device").as_string();
  serial_baud_ = this->get_parameter("serial_baud").as_int();
  serial_data_bits_ = this->get_parameter("serial_data_bits").as_int();
  serial_stop_bits_ = this->get_parameter("serial_stop_bits").as_int();
  serial_parity_ = tip_detail::to_lower_copy(this->get_parameter("serial_parity").as_string());
  target_labels_ = this->get_parameter("target_labels").as_string_array();
  grab_min_width_ratio_ = this->get_parameter("grab_min_width_ratio").as_double();
  ok_stable_frames_ = this->get_parameter("ok_stable_frames").as_int();
  center_ratio_ = this->get_parameter("center_ratio").as_double();
  small_ratio_ = this->get_parameter("small_ratio").as_double();
  medium_ratio_ = this->get_parameter("medium_ratio").as_double();

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
  if (serial_baud_ <= 0) {
    throw std::runtime_error("serial_baud must be > 0");
  }
  if (serial_data_bits_ != 8) {
    throw std::runtime_error("rc26_serial only supports serial_data_bits=8");
  }
  if (serial_stop_bits_ != 1) {
    throw std::runtime_error("rc26_serial only supports serial_stop_bits=1");
  }
  if (serial_parity_ != "none") {
    throw std::runtime_error("rc26_serial only supports serial_parity=none");
  }
  if (grab_min_width_ratio_ <= 0.0 || grab_min_width_ratio_ > 1.0) {
    throw std::runtime_error("grab_min_width_ratio must be in (0,1]");
  }
  if (ok_stable_frames_ <= 0) {
    throw std::runtime_error("ok_stable_frames must be > 0");
  }
  if (center_ratio_ <= 0.0 || center_ratio_ >= 1.0) {
    throw std::runtime_error("center_ratio must be in (0,1)");
  }
  if (small_ratio_ <= center_ratio_ || small_ratio_ >= 1.0) {
    throw std::runtime_error("small_ratio must be in (center_ratio,1)");
  }
  if (medium_ratio_ <= small_ratio_ || medium_ratio_ >= 1.0) {
    throw std::runtime_error("medium_ratio must be in (small_ratio,1)");
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
    RCLCPP_ERROR(get_logger(), "Model profile '%s' has no labels.", model_id_.c_str());
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
        get_logger(), "Configured target label '%s' not found in model profile labels.",
        label.c_str());
      return false;
    }
    const int class_id = static_cast<int>(std::distance(class_names_.begin(), it));
    if (seen_ids.find(class_id) != seen_ids.end()) {
      RCLCPP_ERROR(
        get_logger(), "Configured target label '%s' maps to duplicated class_id=%d.",
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
  RCLCPP_INFO(get_logger(), "Target label mapping: %s", oss.str().c_str());
  return true;
}

bool TipVisionTestNode::is_target_class(int class_id) const
{
  return std::find(target_class_ids_.begin(), target_class_ids_.end(), class_id) !=
         target_class_ids_.end();
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

std::optional<TipVisionTestNode::TargetCandidate> TipVisionTestNode::select_primary_target(
  const std::vector<rc26_vision::Detection> & detections) const
{
  std::optional<TargetCandidate> best;
  int best_area = -1;

  for (const auto & det : detections) {
    if (!is_target_class(det.class_id)) {
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
    const int area = box.area();
    if (!best.has_value() || area > best_area ||
      (area == best_area && det.score > best->score))
    {
      best = TargetCandidate{box, det.class_id, det.score};
      best_area = area;
    }
  }

  return best;
}

TipVisionTestNode::OffsetDecision TipVisionTestNode::classify_offset(int offset_px, int ref_w) const
{
  const int abs_offset = std::abs(offset_px);
  const int th_center = std::max(1, static_cast<int>(std::round(ref_w * center_ratio_)));
  const int th_small = std::max(th_center + 1, static_cast<int>(std::round(ref_w * small_ratio_)));
  const int th_medium = std::max(th_small + 1, static_cast<int>(std::round(ref_w * medium_ratio_)));

  if (abs_offset <= th_center) {
    return {0x00, 0x00};
  }

  if (offset_px < 0) {
    if (abs_offset <= th_small) {
      return {0x01, 0x01};
    }
    if (abs_offset <= th_medium) {
      return {0x01, 0x02};
    }
    return {0x01, 0x03};
  }

  if (abs_offset <= th_small) {
    return {0x02, 0x01};
  }
  if (abs_offset <= th_medium) {
    return {0x02, 0x02};
  }
  return {0x02, 0x03};
}

}  // namespace rc26_vision::test

// ---- tip_vision_test_node_serial.cpp ----

#include <vector>

namespace rc26_vision::test {

bool TipVisionTestNode::is_serial_open() const
{
  return serial_driver_ && serial_driver_->isOpen();
}

void TipVisionTestNode::close_serial_driver()
{
  if (serial_driver_) {
    serial_driver_->close();
  }
}

bool TipVisionTestNode::open_serial_port()
{
  if (!serial_enable_) {
    return false;
  }

  if (!serial_driver_) {
    serial_driver_ = std::make_shared<rc26_decision::SerialDriver>();
    serial_driver_->setReconnectCallback([this]() {
      RCLCPP_INFO(
        this->get_logger(), "rc26_serial reconnected: dev=%s baud=%d",
        serial_device_.c_str(), serial_baud_);
    });
    serial_driver_->setReconnectFailedCallback([this]() {
      RCLCPP_ERROR(
        this->get_logger(), "rc26_serial reconnect exhausted: dev=%s baud=%d",
        serial_device_.c_str(), serial_baud_);
    });
  }

  if (serial_driver_->isOpen()) {
    return true;
  }

  if (!serial_driver_->open(serial_device_, serial_baud_)) {
    RCLCPP_WARN(
      get_logger(), "rc26_serial open failed on '%s': %s",
      serial_device_.c_str(), serial_driver_->lastError().c_str());
    return false;
  }

  RCLCPP_INFO(
    get_logger(), "rc26_serial opened: dev=%s baud=%d cmd=0x%02X payload_len=5",
    serial_device_.c_str(), serial_baud_, static_cast<unsigned int>(kTipVisionCommand));
  return true;
}

void TipVisionTestNode::serial_reconnect_if_needed()
{
  if (!serial_enable_ || is_serial_open()) {
    return;
  }

  const auto now = std::chrono::steady_clock::now();
  if (now - last_serial_open_attempt_tp_ < serial_reconnect_interval_) {
    return;
  }

  last_serial_open_attempt_tp_ = now;
  (void)open_serial_port();
}

bool TipVisionTestNode::write_serial_packet(const std::array<uint8_t, 5> & packet, uint8_t & out_seq)
{
  if (!serial_enable_) {
    return false;
  }

  serial_reconnect_if_needed();
  if (!is_serial_open()) {
    return false;
  }

  const std::vector<uint8_t> payload(packet.begin(), packet.end());
  if (!serial_driver_->sendCommandNoAck(kTipVisionCommand, payload, out_seq)) {
    RCLCPP_WARN(
      get_logger(), "rc26_serial send failed on '%s': %s",
      serial_device_.c_str(), serial_driver_->lastError().c_str());
    return false;
  }
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
      "Camera index %d opened but failed to read the first frame; releasing this candidate.",
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
      "Camera path '%s' opened but failed to read the first frame; releasing this candidate.",
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
    RCLCPP_INFO(get_logger(), "Trying preferred camera path '%s'...", camera_device_.c_str());
    opened = open_camera_by_path(camera_device_, opened_source);
  } else {
    RCLCPP_INFO(get_logger(), "Trying preferred camera index %d...", camera_index_);
    opened = open_camera_by_index(camera_index_, opened_source);
  }

  if (!opened && auto_scan_camera_) {
    RCLCPP_WARN(
      get_logger(),
      "Preferred camera did not yield frames; auto_scan_camera=true, scanning other /dev/video* devices.");
    const std::vector<std::string> candidates = discover_video_devices();
    for (const auto & candidate : candidates) {
      if (!camera_device_.empty() && candidate == camera_device_) {
        continue;
      }
      const int candidate_index = parse_video_index_from_path(candidate);
      RCLCPP_INFO(get_logger(), "Trying fallback camera candidate %s...", candidate.c_str());
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
      get_logger(), "Failed to open a working camera (camera_index=%d camera_device='%s').",
      camera_index_, camera_device_.c_str());
    return false;
  }

  const int actual_w = static_cast<int>(camera_.get(cv::CAP_PROP_FRAME_WIDTH));
  const int actual_h = static_cast<int>(camera_.get(cv::CAP_PROP_FRAME_HEIGHT));
  const int actual_fps = static_cast<int>(camera_.get(cv::CAP_PROP_FPS));
  selected_camera_source_ = opened_source;

  RCLCPP_INFO(
    get_logger(), "Camera opened on %s (backend=%s): actual=%dx%d@%dfps requested=%dx%d@%d",
    selected_camera_source_.c_str(), camera_.getBackendName().c_str(), actual_w, actual_h, actual_fps,
    camera_width_, camera_height_, camera_fps_);
  return true;
}

}  // namespace rc26_vision::test

// ---- tip_vision_test_node_inference.cpp ----

#include "rc26_vision/runtime/inference_engine_factory.hpp"
#include "rc26_vision/runtime/profile_loader.hpp"

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

  local_result.primary_target = select_primary_target(detections);
  local_result.has_target = local_result.primary_target.has_value();
  if (local_result.has_target) {
    local_result.box_w = local_result.primary_target->box.width;
    local_result.box_cx =
      local_result.primary_target->box.x + local_result.primary_target->box.width / 2;
    local_result.offset_px = local_result.box_cx - frame_bgr.cols / 2;
    local_result.class_id = local_result.primary_target->class_id;
    const int tracking_ref_w = std::max(
      1, static_cast<int>(std::round(static_cast<double>(frame_bgr.cols) * kMainPyRefWidthRatio)));
    const OffsetDecision offset_decision = classify_offset(local_result.offset_px, tracking_ref_w);
    local_result.dir_code = offset_decision.dir_code;
    local_result.amp_code = offset_decision.amp_code;
    const int grab_w_th = std::max(
      2, static_cast<int>(std::round(static_cast<double>(tracking_ref_w) * grab_min_width_ratio_)));
    local_result.grab_ready_now =
      local_result.dir_code == 0x00 &&
      local_result.amp_code == 0x00 &&
      local_result.box_w >= grab_w_th;
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
        get_logger(), *get_clock(), 2000, "Inference failed through shared engine.");
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
    RCLCPP_ERROR(get_logger(), "Failed to start async inference worker: %s", ex.what());
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
    "Detection policy: model_id=%s single_target_mode=%s max_categories=%d use_predicted_label=%s",
    model_id_.c_str(), single_target_mode_ ? "true" : "false", max_categories_,
    use_predicted_label_ ? "true" : "false");

  if (!init_camera()) {
    return false;
  }

  stable_ok_count_ = 0;
  last_serial_open_attempt_tp_ = std::chrono::steady_clock::time_point::min();
  RCLCPP_INFO(
    get_logger(),
    "Serial control uses rc26_serial cmd=0x%02X payload: grab_ready | dir_code | amp_code | "
    "ts16_lo | ts16_hi",
    static_cast<unsigned int>(kTipVisionCommand));
  if (serial_enable_) {
    (void)open_serial_port();
  }

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
        get_logger(), "Vision model_id '%s' not found in %s",
        model_id_.c_str(), vision_config_file_.c_str());
      return false;
    }

    model_profile_ = profile_it->second;
    class_names_ = model_profile_.labels;
    engine_ = rc26_vision::createInferenceEngine(model_profile_);
    RCLCPP_INFO(
      get_logger(), "Tip inference uses shared engine profile '%s': model=%s labels=%zu",
      model_profile_.id.c_str(), model_profile_.model_path.c_str(), class_names_.size());
    return true;
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(get_logger(), "Failed to initialize shared inference engine: %s", ex.what());
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

void TipVisionTestNode::draw_serial_overlay(cv::Mat & frame_bgr, const SerialOverlayInfo & info) const
{
  std::vector<std::string> lines;
  lines.reserve(4U);

  const char * serial_state = !info.serial_enabled ? "OFF" : (info.serial_open ? "OPEN" : "DOWN");
  const char * tx_state = info.serial_enabled ? (info.tx_sent ? "SENT" : "DROP") : "PREVIEW";

  {
    std::ostringstream ss;
    ss << "SER " << serial_state << " dev=" << serial_device_ << " tx=" << tx_state;
    lines.push_back(ss.str());
  }

  {
    std::ostringstream ss;
    ss << "PACK cmd=" << tip_detail::byte_to_hex(info.command_id)
       << " len=" << static_cast<unsigned int>(info.packet_size)
       << " seq=" << static_cast<unsigned int>(info.seq)
       << " ts=0x" << std::uppercase << std::hex << std::setfill('0') << std::setw(4)
       << static_cast<unsigned int>(info.ts16)
       << std::dec << " target=" << (info.has_target ? "YES" : "NO");
    lines.push_back(ss.str());
  }

  {
    std::ostringstream ss;
    ss << "CTRL " << info.label
       << " off=" << info.offset_px
       << " dir=" << tip_detail::byte_to_hex(info.dir_code)
       << " amp=" << tip_detail::byte_to_hex(info.amp_code)
       << " grab=" << (info.grab_ready ? "OK" : "WAIT")
       << " " << info.stable_ok_count << "/" << ok_stable_frames_;
    lines.push_back(ss.str());
  }

  {
    std::ostringstream ss;
    ss << "PAY " << info.packet_hex;
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
    info.serial_enabled ?
    (info.serial_open ? cv::Scalar(35, 45, 35) : cv::Scalar(20, 40, 90)) :
    cv::Scalar(55, 55, 55);
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
    std::fprintf(stderr, "Fatal: %s\n", ex.what());
    rc = 2;
  } catch (...) {
    std::fprintf(stderr, "Unknown fatal error\n");
    rc = 3;
  }

  std::fflush(nullptr);
  // ROS logging and Aidlite teardown are unstable on this target during process
  // shutdown. Exit the process directly after the run loop finishes.
  std::_Exit(rc);
}
