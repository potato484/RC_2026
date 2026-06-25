// RC2026 融合里程计主节点
// 整合CAN/Wheel里程计和速度发送功能（当前默认单口 MCU）
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "rc26_merge_odom/can/can_odom.hpp"
#include "rc26_merge_odom/pose/pose_sender.hpp"
#include "rc26_merge_odom/transport/mechanism_transport_bridge.hpp"
#include "rc26_merge_odom/wheel/wheel_odom.hpp"
#include "rc26_serial/serial_driver.hpp"

namespace {

bool serialPortDisabled(const std::string& port) {
    return port.empty() || port == "__disabled__" || port == "disabled";
}
}  // namespace

class MergeOdomNode : public rclcpp::Node {
public:
    MergeOdomNode() : Node("merge_odom_node") {
        // 里程计源选择
        this->declare_parameter("use_can_odom", false);
        this->declare_parameter("require_odom_source", false);

        // CAN里程计参数
        this->declare_parameter("can_interface", "can0");
        this->declare_parameter("wheel_radius", 0.07625);
        this->declare_parameter("wheel_base", 0.62326);
        this->declare_parameter("track_width", 0.7);
        this->declare_parameter("gear_ratio", 3591.0 / 187.0);
        this->declare_parameter("can_publish_rate_hz", 50);
        this->declare_parameter("can_odom_topic", "Can_Odom");
        this->declare_parameter("wheel_odom_topic", "wheel_odom");
        this->declare_parameter("odom_frame", "odom");
        this->declare_parameter("base_frame", "base_link");
        this->declare_parameter("data_timeout_ms", 100.0);

        // 串口参数（当前默认 target 单口同时承载目标下发、机构 transport 与 WheelOdom）
        this->declare_parameter("feedback_serial_port", "__disabled__");
        this->declare_parameter("target_serial_port", "/dev/ttyUSB0");
        this->declare_parameter("baudrate", 1000000);

        // 速度发送参数
        this->declare_parameter("cmd_vel_topic", "cmd_vel");
        this->declare_parameter("merge_odom_topic", "merge_odom");
        this->declare_parameter("feedback_send_rate_hz", 50);
        this->declare_parameter("target_send_rate_hz", 50);
        this->declare_parameter("cmd_vel_timeout_ms", 200);

        // 速度保护参数 (PoseSender)
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

        // 自适应协方差参数 (WheelOdom)
        this->declare_parameter("slip_enable", true);
        this->declare_parameter("slip_threshold", 0.5);
        this->declare_parameter("slip_k_acc", 0.5);
        this->declare_parameter("cov_nominal_v", 0.01);
        this->declare_parameter("cov_nominal_wz", 0.03);
        this->declare_parameter("cov_slip_v", 0.5);
        this->declare_parameter("cov_slip_wz", 0.5);
        this->declare_parameter("recovery_tau_s", 0.5);

        // 获取里程计源配置
        bool use_can_odom = this->get_parameter("use_can_odom").as_bool();
        bool require_odom_source = this->get_parameter("require_odom_source").as_bool();

        // 初始化串口
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
            if (use_can_odom) {
                RCLCPP_WARN(this->get_logger(), "独立反馈串口已禁用；速度反馈将不可用");
            } else {
                RCLCPP_INFO(this->get_logger(),
                            "独立反馈串口已禁用；WheelOdom 将使用目标串口单链路，速度反馈将不可用");
            }
        } else {
            feedback_serial_ = std::make_shared<rc26_decision::SerialDriver>();
            feedback_ok = feedback_serial_->open(feedback_port, baudrate);
            if (!feedback_ok) {
                RCLCPP_WARN(this->get_logger(), "反馈串口打开失败: %s", feedback_port.c_str());
            }
        }

        bool target_ok = false;
        if (target_disabled) {
            RCLCPP_WARN(this->get_logger(), "目标串口已禁用；POSE_TARGET / mechanism transport 将不可用");
        } else {
            target_serial_ = std::make_shared<rc26_decision::SerialDriver>();
            target_ok = target_serial_->open(target_port, baudrate);
            if (!target_ok) {
                RCLCPP_WARN(this->get_logger(), "目标串口打开失败: %s", target_port.c_str());
            }
        }

        const bool wheel_can_use_target_serial = !use_can_odom && !feedback_ok && target_ok;
        if (!use_can_odom && require_odom_source && !feedback_ok && !wheel_can_use_target_serial) {
            RCLCPP_FATAL(this->get_logger(),
                         "require_odom_source=true，但选中的 WheelOdom 源不可用；"
                         "target_serial_port=%s，feedback_serial_port=%s",
                         target_port.c_str(), feedback_port.c_str());
            throw std::runtime_error("require_odom_source=true but selected WheelOdom serial is unavailable");
        }

        if (!feedback_ok && !target_ok) {
            throw std::runtime_error("feedback_serial_port 与 target_serial_port 均不可用");
        }

        // 获取共享参数
        const double wheel_base = this->get_parameter("wheel_base").as_double();
        const double track_width = this->get_parameter("track_width").as_double();
        const int publish_rate_hz = this->get_parameter("can_publish_rate_hz").as_int();
        const std::string can_odom_topic = this->get_parameter("can_odom_topic").as_string();
        const std::string wheel_odom_topic = this->get_parameter("wheel_odom_topic").as_string();
        const std::string odom_topic = use_can_odom ? can_odom_topic : wheel_odom_topic;
        const std::string merge_odom_topic = this->get_parameter("merge_odom_topic").as_string();
        const std::string odom_frame = this->get_parameter("odom_frame").as_string();
        const std::string base_frame = this->get_parameter("base_frame").as_string();
        const double data_timeout_ms = this->get_parameter("data_timeout_ms").as_double();

        RCLCPP_INFO(this->get_logger(),
                    "里程计源=%s raw_output=%s pose_feedback=%s require_odom_source=%s",
                    use_can_odom ? "can_odom" : "wheel_odom", odom_topic.c_str(), merge_odom_topic.c_str(),
                    require_odom_source ? "true" : "false");

        // 根据配置初始化里程计
        if (use_can_odom) {
            rc26_merge_odom::CanOdom::Config can_config;
            can_config.can_interface = this->get_parameter("can_interface").as_string();
            can_config.wheel_radius = this->get_parameter("wheel_radius").as_double();
            can_config.wheel_base = wheel_base;
            can_config.track_width = track_width;
            can_config.gear_ratio = this->get_parameter("gear_ratio").as_double();
            can_config.publish_rate_hz = publish_rate_hz;
            can_config.odom_topic = odom_topic;
            can_config.odom_frame = odom_frame;
            can_config.base_frame = base_frame;
            can_config.data_timeout_ms = data_timeout_ms;
            can_config.imu_topic = this->get_parameter("imu_topic").as_string();
            can_config.slip_enable = this->get_parameter("slip_enable").as_bool();
            can_config.slip_threshold = this->get_parameter("slip_threshold").as_double();
            can_config.slip_k_acc = this->get_parameter("slip_k_acc").as_double();
            can_config.cov_nominal_v = this->get_parameter("cov_nominal_v").as_double();
            can_config.cov_nominal_wz = this->get_parameter("cov_nominal_wz").as_double();
            can_config.cov_slip_v = this->get_parameter("cov_slip_v").as_double();
            can_config.cov_slip_wz = this->get_parameter("cov_slip_wz").as_double();
            can_config.recovery_tau_s = this->get_parameter("recovery_tau_s").as_double();

            can_odom_ = std::make_unique<rc26_merge_odom::CanOdom>(*this, can_config);
            if (!can_odom_->isReady()) {
                throw std::runtime_error("CanOdom 初始化失败");
            }
            RCLCPP_INFO(this->get_logger(), "使用 CAN 里程计");
        } else {
            auto wheel_serial = feedback_ok ? feedback_serial_ : target_serial_;
            const bool wheel_uses_target_serial = wheel_serial && wheel_serial == target_serial_;
            if (!wheel_serial || !wheel_serial->isOpen()) {
                RCLCPP_WARN(this->get_logger(), "WheelOdom 串口不可用，跳过 WheelOdom");
            } else {
                rc26_merge_odom::WheelOdom::Config wheel_config;
                wheel_config.wheel_base = wheel_base;
                wheel_config.track_width = track_width;
                wheel_config.publish_rate_hz = publish_rate_hz;
                wheel_config.odom_topic = odom_topic;
                wheel_config.odom_frame = odom_frame;
                wheel_config.base_frame = base_frame;
                wheel_config.data_timeout_ms = data_timeout_ms;
                wheel_config.imu_topic = this->get_parameter("imu_topic").as_string();
                wheel_config.slip_enable = this->get_parameter("slip_enable").as_bool();
                wheel_config.slip_threshold = this->get_parameter("slip_threshold").as_double();
                wheel_config.slip_k_acc = this->get_parameter("slip_k_acc").as_double();
                wheel_config.cov_nominal_v = this->get_parameter("cov_nominal_v").as_double();
                wheel_config.cov_nominal_wz = this->get_parameter("cov_nominal_wz").as_double();
                wheel_config.cov_slip_v = this->get_parameter("cov_slip_v").as_double();
                wheel_config.cov_slip_wz = this->get_parameter("cov_slip_wz").as_double();
                wheel_config.recovery_tau_s = this->get_parameter("recovery_tau_s").as_double();

                wheel_odom_ = std::make_unique<rc26_merge_odom::WheelOdom>(
                    *this, wheel_serial, wheel_config, !wheel_uses_target_serial);
                if (!wheel_odom_->isReady()) {
                    throw std::runtime_error("WheelOdom 初始化失败");
                }
                RCLCPP_INFO(this->get_logger(), "使用 Wheel 里程计 (%s: %s)",
                            wheel_uses_target_serial ? "目标串口单链路" : "反馈串口",
                            wheel_uses_target_serial ? target_port.c_str() : feedback_port.c_str());
            }
        }

        // 初始化速度发送
        if (feedback_ok || target_ok) {
            rc26_merge_odom::PoseSender::Config pose_config;
            pose_config.cmd_vel_topic = this->get_parameter("cmd_vel_topic").as_string();
            pose_config.odom_topic = merge_odom_topic;
            pose_config.imu_topic = this->get_parameter("imu_topic").as_string();
            pose_config.feedback_send_rate_hz = this->get_parameter("feedback_send_rate_hz").as_int();
            pose_config.target_send_rate_hz = this->get_parameter("target_send_rate_hz").as_int();
            pose_config.cmd_vel_timeout_ms = this->get_parameter("cmd_vel_timeout_ms").as_int();
            pose_config.v_max_mps = static_cast<float>(this->get_parameter("v_max_mps").as_double());
            pose_config.w_max_rps = static_cast<float>(this->get_parameter("w_max_rps").as_double());
            pose_config.a_max_mps2 = static_cast<float>(this->get_parameter("a_max_mps2").as_double());
            pose_config.alpha_max_rps2 = static_cast<float>(this->get_parameter("alpha_max_rps2").as_double());
            pose_config.imu_gate_enable = this->get_parameter("imu_gate_enable").as_bool();
            pose_config.imu_gate_ema_alpha = static_cast<float>(this->get_parameter("imu_gate_ema_alpha").as_double());
            pose_config.imu_gate_chi2_threshold =
                static_cast<float>(this->get_parameter("imu_gate_chi2_threshold").as_double());
            pose_config.accel_agree_threshold_mps2 =
                static_cast<float>(this->get_parameter("accel_agree_threshold_mps2").as_double());
            pose_config.spike_freeze_duration_ms = this->get_parameter("spike_freeze_duration_ms").as_int();
            pose_config.spike_decay_tau_s = static_cast<float>(this->get_parameter("spike_decay_tau_s").as_double());
            pose_config.governor_enable = this->get_parameter("governor_enable").as_bool();
            pose_config.governor_lambda = static_cast<float>(this->get_parameter("governor_lambda").as_double());
            pose_config.dob_enable = this->get_parameter("dob_enable").as_bool();
            pose_config.dob_lpf_hz = static_cast<float>(this->get_parameter("dob_lpf_hz").as_double());
            pose_config.dob_kd = static_cast<float>(this->get_parameter("dob_kd").as_double());
            pose_config.latency_comp_enable = this->get_parameter("latency_comp_enable").as_bool();
            pose_config.latency_comp_s = static_cast<float>(this->get_parameter("latency_comp_s").as_double());
            pose_config.stats_log_enable = this->get_parameter("stats_log_enable").as_bool();
            pose_config.imu_gate_log_enable = this->get_parameter("imu_gate_log_enable").as_bool();

            pose_sender_ =
                std::make_unique<rc26_merge_odom::PoseSender>(*this, feedback_serial_, target_serial_, pose_config);
        }

        const bool shared_target_serial_dispatch =
            !use_can_odom && wheel_odom_ && target_serial_ && target_serial_->isOpen() && !feedback_ok;
        mechanism_transport_bridge_ = std::make_unique<rc26_merge_odom::MechanismTransportBridge>(
            *this, target_serial_, !shared_target_serial_dispatch);

        if (shared_target_serial_dispatch) {
            target_serial_->setReceiveCallback([this](uint8_t seq, uint8_t cmd, const std::vector<uint8_t>& payload) {
                if (wheel_odom_) {
                    wheel_odom_->handleSerialFrame(cmd, payload);
                }
                if (mechanism_transport_bridge_) {
                    mechanism_transport_bridge_->handleSerialFrame(seq, cmd, payload);
                }
            });
            RCLCPP_INFO(this->get_logger(),
                        "目标串口单链路分发已启用：ODOM_DATA -> WheelOdom，其它业务反馈 -> mechanism transport");
        }

        if (feedback_ok && target_ok) {
            RCLCPP_INFO(this->get_logger(), "融合里程计节点启动 (双串口模式)");
        } else if (target_ok) {
            RCLCPP_INFO(this->get_logger(), "融合里程计节点启动 (目标串口单链路模式)");
        } else {
            RCLCPP_INFO(this->get_logger(), "融合里程计节点启动 (仅反馈链路模式)");
        }
    }

    ~MergeOdomNode() {
        if (target_serial_) {
            target_serial_->setReceiveCallback({});
        }
        if (feedback_serial_ && feedback_serial_ != target_serial_) {
            feedback_serial_->setReceiveCallback({});
        }
    }

private:
    std::unique_ptr<rc26_merge_odom::CanOdom> can_odom_;
    std::unique_ptr<rc26_merge_odom::WheelOdom> wheel_odom_;
    std::shared_ptr<rc26_decision::SerialDriver> feedback_serial_;
    std::shared_ptr<rc26_decision::SerialDriver> target_serial_;
    std::unique_ptr<rc26_merge_odom::PoseSender> pose_sender_;
    std::unique_ptr<rc26_merge_odom::MechanismTransportBridge> mechanism_transport_bridge_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    int exit_code = 0;
    try {
        auto node = std::make_shared<MergeOdomNode>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("merge_odom_node"), "Exception: %s", e.what());
        exit_code = 1;
    }
    rclcpp::shutdown();
    return exit_code;
}
