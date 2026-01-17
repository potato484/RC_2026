#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include <mutex>
#include <cmath>
#include <algorithm>

class rc26_telecontrol : public rclcpp::Node
{
private:
    // ROS
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    // 参数
    double v_linear_{1.0};
    double v_angular_{2.0};
    double joy_deadzone_{0.1};
    double smoothing_alpha_{0.2};  // 平滑系数（0.0~1.0）
    
    // Joy 状态
    sensor_msgs::msg::Joy::SharedPtr latest_joy_;
    std::mutex joy_mutex_;
    
    // 当前平滑后的速度输出
    geometry_msgs::msg::Twist current_output_twist_{};
    
    // 事件驱动发布辅助
    bool published_zero_{true};  // 启动期不重复发0
    static constexpr double EPS = 1e-6;
    
    // 手柄回调
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(joy_mutex_);
        latest_joy_ = msg;
    }
    
    // 死区处理（线性映射）
    static double apply_deadzone(double value, double deadzone)
    {
        if (std::abs(value) < deadzone) return 0.0;
        double sign = (value >= 0.0) ? 1.0 : -1.0;
        return sign * (std::abs(value) - deadzone) / (1.0 - deadzone);
    }
    
    static bool is_zero_twist(const geometry_msgs::msg::Twist &t, double tol = EPS)
    {
        return std::abs(t.linear.x) < tol &&
               std::abs(t.linear.y) < tol &&
               std::abs(t.angular.z) < tol;
    }
    
    // 定时器回调：摇杆控制逻辑
    void timer_callback()
    {
        // 复制手柄状态，缩小锁作用域
        sensor_msgs::msg::Joy::SharedPtr joy_copy;
        {
            std::lock_guard<std::mutex> lock(joy_mutex_);
            joy_copy = latest_joy_;
        }
        
        // 构建目标速度
        geometry_msgs::msg::Twist target_twist;
        
        if (joy_copy) {
            // 左摇杆：前后 (axes[1])
            if (joy_copy->axes.size() > 1) {
                double left_stick_forward = apply_deadzone(joy_copy->axes[1], joy_deadzone_);
                target_twist.linear.x = left_stick_forward * v_linear_;  // ✅ 修正：去掉负号
            }
            
            // 左摇杆：左右 (axes[0])
            if (joy_copy->axes.size() > 0) {
                double left_stick_right = apply_deadzone(joy_copy->axes[0], joy_deadzone_);
                target_twist.linear.y = left_stick_right * v_linear_;
            }
            
            // 右摇杆：左右 (axes[3]) 控制旋转
            if (joy_copy->axes.size() > 3) {
                double right_stick_left = apply_deadzone(joy_copy->axes[3], joy_deadzone_);
                target_twist.angular.z = right_stick_left * v_angular_;  // ✅ 修正：去掉负号
            }
        }
        
        // 限幅到最大速度
        target_twist.linear.x = std::clamp(target_twist.linear.x, -v_linear_, v_linear_);
        target_twist.linear.y = std::clamp(target_twist.linear.y, -v_linear_, v_linear_);
        target_twist.angular.z = std::clamp(target_twist.angular.z, -v_angular_, v_angular_);
        
        // 平滑过渡：指数加权移动平均
        current_output_twist_.linear.x = 
            smoothing_alpha_ * target_twist.linear.x + 
            (1.0 - smoothing_alpha_) * current_output_twist_.linear.x;
        
        current_output_twist_.linear.y = 
            smoothing_alpha_ * target_twist.linear.y + 
            (1.0 - smoothing_alpha_) * current_output_twist_.linear.y;
        
        current_output_twist_.angular.z = 
            smoothing_alpha_ * target_twist.angular.z + 
            (1.0 - smoothing_alpha_) * current_output_twist_.angular.z;
        
        // 事件驱动发布：非零则发布；零则只发布一次停止
        if (is_zero_twist(current_output_twist_)) {
            if (!published_zero_) {
                geometry_msgs::msg::Twist stop_msg;
                pub_->publish(stop_msg);
                published_zero_ = true;
            }
            return;  // 零状态不重复发
        } else {
            pub_->publish(current_output_twist_);
            published_zero_ = false;
        }
    }
public:
    rc26_telecontrol() : Node("rc26_telecontrol")
    {
        // 声明参数
        this->declare_parameter<double>("v_linear", 1.0);
        this->declare_parameter<double>("v_angular", 2.0);
        this->declare_parameter<double>("joy_deadzone", 0.1);
        this->declare_parameter<double>("smoothing_alpha", 0.2);
        
        // 获取参数
        this->get_parameter("v_linear", v_linear_);
        this->get_parameter("v_angular", v_angular_);
        this->get_parameter("joy_deadzone", joy_deadzone_);
        this->get_parameter("smoothing_alpha", smoothing_alpha_);
        
        // 约束 smoothing_alpha 在 [0.0, 1.0]
        smoothing_alpha_ = std::clamp(smoothing_alpha_, 0.0, 1.0);
        
        // QoS
        rclcpp::QoS qos_profile(10);
        qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
        qos_profile.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
        qos_profile.history(RMW_QOS_POLICY_HISTORY_KEEP_LAST);
        
        // Pub/Sub
        pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", qos_profile);
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", qos_profile, std::bind(&rc26_telecontrol::joy_callback, this, std::placeholders::_1));

        // 定时器 50Hz
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&rc26_telecontrol::timer_callback, this));

        // 启动日志
        RCLCPP_INFO(this->get_logger(), "rc26_telecontrol (dual-stick joystick) ready.");
        RCLCPP_INFO(this->get_logger(), 
                    "v_linear=%.2f m/s, v_angular=%.2f rad/s, deadzone=%.2f, smoothing_alpha=%.2f",
                    v_linear_, v_angular_, joy_deadzone_, smoothing_alpha_);
        RCLCPP_INFO(this->get_logger(), 
                    "Left Stick: forward/backward(axes[1]), left/right(axes[0])");
        RCLCPP_INFO(this->get_logger(), 
                    "Right Stick: left/right(axes[3]) for rotation");
    }
    
    ~rc26_telecontrol()
    {
        // 退出时发一次停止
        geometry_msgs::msg::Twist stop_msg;
        pub_->publish(stop_msg);
        RCLCPP_INFO(this->get_logger(), "rc26_telecontrol stopped.");
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<rc26_telecontrol>());
    rclcpp::shutdown();
    return 0;
}