// RC2026 速度发送独立节点
// 双串口架构：反馈速度 + 目标速度
#include <rclcpp/rclcpp.hpp>

#include "rc26_merge_odom/pose/pose_sender.hpp"
#include "rc26_serial/serial_driver.hpp"

class PoseSenderNode : public rclcpp::Node {
public:
    PoseSenderNode() : Node("pose_sender_node") {
        this->declare_parameter("feedback_serial_port", "/dev/ttyUSB0");
        this->declare_parameter("target_serial_port", "/dev/ttyUSB1");
        this->declare_parameter("baudrate", 1000000);
        this->declare_parameter("cmd_vel_topic", "cmd_vel");
        this->declare_parameter("odom_topic", "merge_odom");
        this->declare_parameter("feedback_send_rate_hz", 50);
        this->declare_parameter("target_send_rate_hz", 25);
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
        this->declare_parameter("terrain_speed_limit_topic", "");
        this->declare_parameter("terrain_speed_limit_timeout_ms", 500);

        std::string feedback_port = this->get_parameter("feedback_serial_port").as_string();
        std::string target_port = this->get_parameter("target_serial_port").as_string();
        int baudrate = this->get_parameter("baudrate").as_int();

        feedback_serial_ = std::make_shared<rc26_decision::SerialDriver>();
        bool feedback_ok = feedback_serial_->open(feedback_port, baudrate);
        if (!feedback_ok) {
            RCLCPP_WARN(this->get_logger(), "反馈串口打开失败: %s", feedback_port.c_str());
        }

        target_serial_ = std::make_shared<rc26_decision::SerialDriver>();
        bool target_ok = target_serial_->open(target_port, baudrate);
        if (!target_ok) {
            RCLCPP_WARN(this->get_logger(), "目标串口打开失败: %s", target_port.c_str());
        }

        if (!feedback_ok && !target_ok) {
            RCLCPP_ERROR(this->get_logger(), "双串口均打开失败，速度发送功能禁用");
            return;
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
        config.terrain_speed_limit_topic = this->get_parameter("terrain_speed_limit_topic").as_string();
        config.terrain_speed_limit_timeout_ms =
            this->get_parameter("terrain_speed_limit_timeout_ms").as_int();

        pose_sender_ = std::make_unique<rc26_merge_odom::PoseSender>(*this, feedback_serial_, target_serial_, config);
    }

private:
    std::shared_ptr<rc26_decision::SerialDriver> feedback_serial_;
    std::shared_ptr<rc26_decision::SerialDriver> target_serial_;
    std::unique_ptr<rc26_merge_odom::PoseSender> pose_sender_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PoseSenderNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
