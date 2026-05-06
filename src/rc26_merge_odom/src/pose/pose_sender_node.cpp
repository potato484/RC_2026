// RC2026 速度发送独立节点
// 双串口架构：反馈速度 + 目标速度
#include <memory>
#include <stdexcept>

#include <rclcpp/rclcpp.hpp>

#include "rc26_merge_odom/pose/pose_sender.hpp"
#include "rc26_merge_odom/transport/mechanism_transport_bridge.hpp"
#include "rc26_serial/serial_driver.hpp"

namespace {

bool serialPortDisabled(const std::string& port) {
    return port.empty() || port == "__disabled__" || port == "disabled";
}

}  // namespace

class PoseSenderNode : public rclcpp::Node {
public:
    PoseSenderNode() : Node("pose_sender_node") {
        this->declare_parameter("feedback_serial_port", "/dev/ttyUSB0");
        this->declare_parameter("target_serial_port", "/dev/ttyUSB1");
        this->declare_parameter("baudrate", 1000000);
        this->declare_parameter("cmd_vel_topic", "cmd_vel");
        this->declare_parameter("odom_topic", "merge_odom");
        this->declare_parameter("feedback_send_rate_hz", 50);
        this->declare_parameter("target_send_rate_hz", 50);
        this->declare_parameter("cmd_vel_timeout_ms", 200);
        this->declare_parameter("imu_topic", "DM_IMU");
        this->declare_parameter("v_max_mps", 2.0);
        this->declare_parameter("w_max_rps", 4.0);
        this->declare_parameter("a_max_mps2", 15.0);
        this->declare_parameter("alpha_max_rps2", 40.0);
        this->declare_parameter("imu_gate_enable", true);
        this->declare_parameter("imu_gate_ema_alpha", 0.98);
        this->declare_parameter("imu_gate_chi2_threshold", 6.635);
        this->declare_parameter("accel_agree_threshold_mps2", 3.0);
        this->declare_parameter("spike_freeze_duration_ms", 100);
        this->declare_parameter("spike_decay_tau_s", 0.3);
        this->declare_parameter("governor_enable", true);
        this->declare_parameter("governor_lambda", 0.2);
        this->declare_parameter("dob_enable", false);
        this->declare_parameter("dob_lpf_hz", 5.0);
        this->declare_parameter("dob_kd", 0.3);
        this->declare_parameter("latency_comp_enable", true);
        this->declare_parameter("latency_comp_s", 0.03);
        this->declare_parameter("stats_log_enable", false);
        this->declare_parameter("imu_gate_log_enable", false);

        std::string feedback_port = this->get_parameter("feedback_serial_port").as_string();
        std::string target_port = this->get_parameter("target_serial_port").as_string();
        int baudrate = this->get_parameter("baudrate").as_int();
        const bool feedback_disabled = serialPortDisabled(feedback_port);
        const bool target_disabled = serialPortDisabled(target_port);

        if (!feedback_disabled && !target_disabled && feedback_port == target_port) {
            RCLCPP_FATAL(this->get_logger(), "feedback_serial_port 与 target_serial_port 指向同一设备: %s",
                         feedback_port.c_str());
            throw std::invalid_argument("feedback_serial_port and target_serial_port must be different");
        }

        bool feedback_ok = false;
        if (feedback_disabled) {
            RCLCPP_INFO(this->get_logger(), "反馈串口已禁用");
        } else {
            feedback_serial_ = std::make_shared<rc26_decision::SerialDriver>();
            feedback_ok = feedback_serial_->open(feedback_port, baudrate);
            if (!feedback_ok) {
                RCLCPP_WARN(this->get_logger(), "反馈串口打开失败: %s", feedback_port.c_str());
            }
        }

        bool target_ok = false;
        if (target_disabled) {
            RCLCPP_INFO(this->get_logger(), "目标串口已禁用");
        } else {
            target_serial_ = std::make_shared<rc26_decision::SerialDriver>();
            target_ok = target_serial_->open(target_port, baudrate);
            if (!target_ok) {
                RCLCPP_WARN(this->get_logger(), "目标串口打开失败: %s", target_port.c_str());
            }
        }

        if (!feedback_ok && !target_ok) {
            RCLCPP_ERROR(this->get_logger(), "双串口均打开失败，速度发送功能禁用");
            throw std::runtime_error("failed to open both pose sender serial ports");
        }

        rc26_merge_odom::PoseSender::Config config;
        config.cmd_vel_topic = this->get_parameter("cmd_vel_topic").as_string();
        config.odom_topic = this->get_parameter("odom_topic").as_string();
        config.imu_topic = this->get_parameter("imu_topic").as_string();
        config.feedback_send_rate_hz = this->get_parameter("feedback_send_rate_hz").as_int();
        config.target_send_rate_hz = this->get_parameter("target_send_rate_hz").as_int();
        config.cmd_vel_timeout_ms = this->get_parameter("cmd_vel_timeout_ms").as_int();
        config.v_max_mps = static_cast<float>(this->get_parameter("v_max_mps").as_double());
        config.w_max_rps = static_cast<float>(this->get_parameter("w_max_rps").as_double());
        config.a_max_mps2 = static_cast<float>(this->get_parameter("a_max_mps2").as_double());
        config.alpha_max_rps2 = static_cast<float>(this->get_parameter("alpha_max_rps2").as_double());
        config.imu_gate_enable = this->get_parameter("imu_gate_enable").as_bool();
        config.imu_gate_ema_alpha = static_cast<float>(this->get_parameter("imu_gate_ema_alpha").as_double());
        config.imu_gate_chi2_threshold =
            static_cast<float>(this->get_parameter("imu_gate_chi2_threshold").as_double());
        config.accel_agree_threshold_mps2 =
            static_cast<float>(this->get_parameter("accel_agree_threshold_mps2").as_double());
        config.spike_freeze_duration_ms = this->get_parameter("spike_freeze_duration_ms").as_int();
        config.spike_decay_tau_s = static_cast<float>(this->get_parameter("spike_decay_tau_s").as_double());
        config.governor_enable = this->get_parameter("governor_enable").as_bool();
        config.governor_lambda = static_cast<float>(this->get_parameter("governor_lambda").as_double());
        config.dob_enable = this->get_parameter("dob_enable").as_bool();
        config.dob_lpf_hz = static_cast<float>(this->get_parameter("dob_lpf_hz").as_double());
        config.dob_kd = static_cast<float>(this->get_parameter("dob_kd").as_double());
        config.latency_comp_enable = this->get_parameter("latency_comp_enable").as_bool();
        config.latency_comp_s = static_cast<float>(this->get_parameter("latency_comp_s").as_double());
        config.stats_log_enable = this->get_parameter("stats_log_enable").as_bool();
        config.imu_gate_log_enable = this->get_parameter("imu_gate_log_enable").as_bool();

        pose_sender_ = std::make_unique<rc26_merge_odom::PoseSender>(*this, feedback_serial_, target_serial_, config);
        mechanism_transport_bridge_ =
            std::make_unique<rc26_merge_odom::MechanismTransportBridge>(*this, target_serial_);
    }

private:
    std::shared_ptr<rc26_decision::SerialDriver> feedback_serial_;
    std::shared_ptr<rc26_decision::SerialDriver> target_serial_;
    std::unique_ptr<rc26_merge_odom::PoseSender> pose_sender_;
    std::unique_ptr<rc26_merge_odom::MechanismTransportBridge> mechanism_transport_bridge_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    int exit_code = 0;
    try {
        auto node = std::make_shared<PoseSenderNode>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("pose_sender_node"), "Exception: %s", e.what());
        exit_code = 1;
    }
    rclcpp::shutdown();
    return exit_code;
}
