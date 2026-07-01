#include <ament_index_cpp/get_package_share_directory.hpp>
#include <behaviortree_cpp/bt_factory.h>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <string>

#include "rc26_decision/decision_failure.hpp"
#include "rc26_decision/mc/mc_area.hpp"
#include "rc26_decision/mf/mf_area.hpp"
#include "rc26_decision/mf_preselection/mf_preselection_flow.hpp"
#include "rc26_decision/navigation/bt_odom_relative_nav.hpp"
#include "rc26_decision/stair/stair_area.hpp"
#include "rc26_decision/team_color.hpp"

namespace rc26_decision {

class DecisionNode : public rclcpp::Node {
public:
  DecisionNode() : Node("rc26_decision") {
    declareParameters();
    initializeBlackboard();
    initializeMerlinState();
    initializeRuntimeState();

    loadMCParams(*this, blackboard_);
    loadOdomRelativeNavParams(*this, blackboard_);
    loadGridHeadingParams(*this, blackboard_);
    loadGridCenterParams(*this, blackboard_);
    loadStairParams(*this, blackboard_);
    loadMfPreselectionParams(*this, blackboard_);

    registerMCAreaNodes(factory_);
    registerMFAreaNodes(factory_);
    registerMfPreselectionNodes(factory_);
    registerOdomNavigationNodes(factory_);
    registerStairNodes(factory_);

    tick_rate_ms_ = this->get_parameter("tick_rate_ms").as_int();
    loadStartupOdomGateParams();
    tree_file_ = this->get_parameter("tree_file").as_string();
    const std::filesystem::path configured_tree(tree_file_);
    tree_path_ = configured_tree.is_absolute()
                     ? configured_tree.lexically_normal().string()
                     : (std::filesystem::path(
                            ament_index_cpp::get_package_share_directory("rc26_decision")) /
                        "behavior_trees" / configured_tree)
                           .lexically_normal()
                           .string();

    RCLCPP_INFO(this->get_logger(), "准备行为树: %s", tree_path_.c_str());
    startWhenRuntimeReady();

    RCLCPP_INFO(this->get_logger(), "决策节点已启动, tick_rate_ms=%d",
                tick_rate_ms_);
  }

private:
  void declareParameters() {
    this->declare_parameter<std::string>("tree_file", "main_tree.xml");
    this->declare_parameter<int>("tick_rate_ms", 100);
    this->declare_parameter<std::string>("team", "red");
    this->declare_parameter<bool>("startup_wait_for_odom", false);
    this->declare_parameter<std::string>("startup_odom_topic", "odom");
    this->declare_parameter<double>("startup_odom_timeout_s", 0.5);
    this->declare_parameter<double>("startup_odom_wait_timeout_s", 15.0);
    this->declare_parameter<double>("startup_odom_min_wait_s", 1.0);
    this->declare_parameter<int>("startup_odom_stable_samples", 10);
    this->declare_parameter<double>("startup_odom_max_linear_speed_mps", 0.03);
    this->declare_parameter<double>("startup_odom_max_angular_speed_radps", 0.05);
  }

  void initializeBlackboard() {
    blackboard_ = BT::Blackboard::create();
    rclcpp::Node *node_ptr = this;
    blackboard_->set("node", node_ptr);
  }

  void initializeMerlinState() {
    const TeamColorRuntime team_runtime =
        resolveTeamColorRuntime(this->get_parameter("team").as_string());
    if (team_runtime.used_fallback) {
      RCLCPP_WARN(this->get_logger(),
                  "team=%s 非法，按 red/+1 红方基准运行",
                  team_runtime.requested.c_str());
    }
    blackboard_->set("team", team_runtime.normalized);
    blackboard_->set("team_mirror_sign", team_runtime.mirror_sign);

    auto merlin_map = std::make_shared<MerlinMapManager>();
    const bool layout_loaded =
        (team_runtime.normalized == "blue") ? merlin_map->initBlueMap()
                                            : merlin_map->initRedMap();
    blackboard_->set("merlin_map", merlin_map);

    if (layout_loaded) {
      RCLCPP_INFO(this->get_logger(),
                  "Cold-start merlin_map initialized for team=%s mirror_sign=%d using %s",
                  team_runtime.normalized.c_str(), team_runtime.mirror_sign,
                  merlin_map->layoutStatus().c_str());
    } else {
      RCLCPP_WARN(this->get_logger(),
                  "Cold-start merlin_map fell back to legacy depths for "
                  "team=%s mirror_sign=%d: %s",
                  team_runtime.normalized.c_str(), team_runtime.mirror_sign,
                  merlin_map->layoutStatus().c_str());
    }
  }

  void initializeRuntimeState() {
    blackboard_->set("stair_climb_done", false);
    blackboard_->set("stair_descend_done", false);
    blackboard_->set("last_action_error_code", 0);
    blackboard_->set("system_error", false);
    blackboard_->set("stable_operation", false);
    blackboard_->set("relative_nav_last_exec_state", std::string("IDLE"));
    blackboard_->set("relative_nav_last_failure_reason", std::string(""));
    blackboard_->set("relative_nav_last_distance_remaining", 0.0);
    clearDecisionFailure(blackboard_);
  }

  void loadBehaviorTree() {
    stopAutoTicking();
    stopStartupOdomGate();
    if (tree_.rootNode()) {
      tree_.haltTree();
    }

    tree_ = factory_.createTreeFromFile(tree_path_, blackboard_);
    clearDecisionFailure(blackboard_);
    terminal_ = false;
  }

  void loadStartupOdomGateParams() {
    startup_wait_for_odom_ =
        this->get_parameter("startup_wait_for_odom").as_bool();
    startup_odom_topic_ =
        this->get_parameter("startup_odom_topic").as_string();
    startup_odom_timeout_s_ =
        this->get_parameter("startup_odom_timeout_s").as_double();
    startup_odom_wait_timeout_s_ =
        this->get_parameter("startup_odom_wait_timeout_s").as_double();
    startup_odom_min_wait_s_ =
        this->get_parameter("startup_odom_min_wait_s").as_double();
    startup_odom_stable_samples_ =
        this->get_parameter("startup_odom_stable_samples").as_int();
    startup_odom_max_linear_speed_mps_ =
        this->get_parameter("startup_odom_max_linear_speed_mps").as_double();
    startup_odom_max_angular_speed_radps_ =
        this->get_parameter("startup_odom_max_angular_speed_radps").as_double();

    if (startup_odom_topic_.empty()) {
      startup_odom_topic_ = "odom";
    }
    if (!std::isfinite(startup_odom_timeout_s_) ||
        startup_odom_timeout_s_ <= 0.0) {
      startup_odom_timeout_s_ = 0.5;
    }
    if (!std::isfinite(startup_odom_wait_timeout_s_) ||
        startup_odom_wait_timeout_s_ <= 0.0) {
      startup_odom_wait_timeout_s_ = 15.0;
    }
    if (!std::isfinite(startup_odom_min_wait_s_) ||
        startup_odom_min_wait_s_ < 0.0) {
      startup_odom_min_wait_s_ = 1.0;
    }
    startup_odom_stable_samples_ = std::max(1, startup_odom_stable_samples_);
    if (!std::isfinite(startup_odom_max_linear_speed_mps_) ||
        startup_odom_max_linear_speed_mps_ < 0.0) {
      startup_odom_max_linear_speed_mps_ = 0.03;
    }
    if (!std::isfinite(startup_odom_max_angular_speed_radps_) ||
        startup_odom_max_angular_speed_radps_ < 0.0) {
      startup_odom_max_angular_speed_radps_ = 0.05;
    }
  }

  void startWhenRuntimeReady() {
    if (!startup_wait_for_odom_) {
      loadBehaviorTree();
      startAutoTicking();
      return;
    }

    startup_odom_stable_count_ = 0;
    startup_odom_start_tp_ = std::chrono::steady_clock::now();
    startup_odom_last_msg_tp_ = {};
    startup_odom_sub_ =
        this->create_subscription<nav_msgs::msg::Odometry>(
            startup_odom_topic_, rclcpp::QoS(rclcpp::KeepLast(10)),
            [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
              handleStartupOdom(msg);
            });
    startup_odom_gate_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        [this]() { tickStartupOdomGate(); });
    RCLCPP_INFO(
        this->get_logger(),
        "行为树启动前等待 odom 稳定: topic=%s stable_samples=%d min_wait=%.2fs wait_timeout=%.2fs fresh_timeout=%.2fs",
        startup_odom_topic_.c_str(), startup_odom_stable_samples_,
        startup_odom_min_wait_s_, startup_odom_wait_timeout_s_,
        startup_odom_timeout_s_);
  }

  void handleStartupOdom(const nav_msgs::msg::Odometry::SharedPtr msg) {
    if (!msg) {
      return;
    }
    startup_odom_last_msg_tp_ = std::chrono::steady_clock::now();

    const auto &linear = msg->twist.twist.linear;
    const auto &angular = msg->twist.twist.angular;
    const double linear_speed =
        std::hypot(linear.x, std::hypot(linear.y, linear.z));
    const double angular_speed =
        std::hypot(angular.x, std::hypot(angular.y, angular.z));
    if (!std::isfinite(linear_speed) || !std::isfinite(angular_speed)) {
      startup_odom_stable_count_ = 0;
      return;
    }

    if (linear_speed <= startup_odom_max_linear_speed_mps_ &&
        angular_speed <= startup_odom_max_angular_speed_radps_) {
      ++startup_odom_stable_count_;
    } else {
      startup_odom_stable_count_ = 0;
      RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 1000,
          "等待 odom 稳定: 当前速度 linear=%.3fm/s angular=%.3frad/s",
          linear_speed, angular_speed);
    }
  }

  void tickStartupOdomGate() {
    const auto now = std::chrono::steady_clock::now();
    const double elapsed_s =
        std::chrono::duration<double>(now - startup_odom_start_tp_).count();

    bool fresh = false;
    if (startup_odom_last_msg_tp_.time_since_epoch().count() > 0) {
      const double age_s =
          std::chrono::duration<double>(now - startup_odom_last_msg_tp_).count();
      fresh = age_s <= startup_odom_timeout_s_;
      if (!fresh) {
        startup_odom_stable_count_ = 0;
      }
    }

    if (fresh && elapsed_s >= startup_odom_min_wait_s_ &&
        startup_odom_stable_count_ >= startup_odom_stable_samples_) {
      RCLCPP_INFO(
          this->get_logger(),
          "odom 已稳定，加载并开始 tick 行为树: topic=%s stable_samples=%d elapsed=%.2fs",
          startup_odom_topic_.c_str(), startup_odom_stable_count_, elapsed_s);
      stopStartupOdomGate();
      loadBehaviorTree();
      startAutoTicking();
      return;
    }

    if (elapsed_s > startup_odom_wait_timeout_s_) {
      terminal_ = true;
      RCLCPP_ERROR(
          this->get_logger(),
          "行为树启动失败: 等待 odom 稳定超时 topic=%s stable_samples=%d/%d elapsed=%.2fs",
          startup_odom_topic_.c_str(), startup_odom_stable_count_,
          startup_odom_stable_samples_, elapsed_s);
      stopStartupOdomGate();
    }
  }

  void stopStartupOdomGate() {
    if (startup_odom_gate_timer_) {
      startup_odom_gate_timer_->cancel();
      startup_odom_gate_timer_.reset();
    }
    startup_odom_sub_.reset();
  }

  void startAutoTicking() {
    stopAutoTicking();
    auto_tick_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(tick_rate_ms_),
        [this]() { (void)tickTree(); });
  }

  void stopAutoTicking() {
    if (auto_tick_timer_) {
      auto_tick_timer_->cancel();
      auto_tick_timer_.reset();
    }
  }

  bool tickTree() {
    if (terminal_) {
      return false;
    }

    const BT::NodeStatus status = tree_.tickOnce();
    terminal_ =
        (status == BT::NodeStatus::SUCCESS || status == BT::NodeStatus::FAILURE);

    if (status == BT::NodeStatus::SUCCESS) {
      RCLCPP_INFO(this->get_logger(), "行为树执行完成: SUCCESS");
    } else if (status == BT::NodeStatus::FAILURE) {
      RCLCPP_ERROR(this->get_logger(),
                   "行为树执行失败: FAILURE，失败原因=%s，行为树=%s",
                   readDecisionFailureDetail(blackboard_).c_str(),
                   tree_path_.c_str());
    }

    if (terminal_) {
      stopAutoTicking();
    }
    return true;
  }

  BT::BehaviorTreeFactory factory_;
  BT::Tree tree_;
  BT::Blackboard::Ptr blackboard_;
  rclcpp::TimerBase::SharedPtr auto_tick_timer_;
  rclcpp::TimerBase::SharedPtr startup_odom_gate_timer_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr startup_odom_sub_;
  std::string tree_file_;
  std::string tree_path_;
  std::string startup_odom_topic_{"odom"};
  int tick_rate_ms_{100};
  int startup_odom_stable_samples_{10};
  int startup_odom_stable_count_{0};
  double startup_odom_timeout_s_{0.5};
  double startup_odom_wait_timeout_s_{15.0};
  double startup_odom_min_wait_s_{1.0};
  double startup_odom_max_linear_speed_mps_{0.03};
  double startup_odom_max_angular_speed_radps_{0.05};
  std::chrono::steady_clock::time_point startup_odom_start_tp_{};
  std::chrono::steady_clock::time_point startup_odom_last_msg_tp_{};
  bool terminal_{false};
  bool startup_wait_for_odom_{false};
};

} // namespace rc26_decision

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rc26_decision::DecisionNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
