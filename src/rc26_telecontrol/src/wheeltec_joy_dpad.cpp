#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include <string>
#include <mutex>
#include <cmath>
#include <algorithm>

class rc26_telecontrol_dpad : public rclcpp::Node
{
private:
    // ROS
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    // 参数
    std::string cmd_vel_topic_{"cmd_vel_teleop"};
    double v_linear_{1.0};
    double v_angular_{2.0};
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
    
    static bool is_zero_twist(const geometry_msgs::msg::Twist &t, double tol = EPS)
    {
        return std::abs(t.linear.x) < tol &&
               std::abs(t.linear.y) < tol &&
               std::abs(t.angular.z) < tol;
    }
    
    // 定时器回调：十字键+按键控制逻辑
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
            // 十字键上下：前后移动 (axes[7])
            // 上: +1, 下: -1, 中位: 0
            if (joy_copy->axes.size() > 7) {
                double dpad_vertical = joy_copy->axes[7];
                target_twist.linear.x = dpad_vertical * v_linear_;
            }
            
            // 十字键左右：左右平移 (axes[6])
            // 右: +1, 左: -1, 中位: 0
            if (joy_copy->axes.size() > 6) {
                double dpad_horizontal = joy_copy->axes[6];
                target_twist.linear.y = dpad_horizontal * v_linear_;
            }
            
            // X键(button[2])和B键(button[1])：旋转控制
            // X键: 左转(逆时针), B键: 右转(顺时针)
            double rotation = 0.0;
            bool x_pressed = (joy_copy->buttons.size() > 2 && joy_copy->buttons[2] == 1);
            bool b_pressed = (joy_copy->buttons.size() > 1 && joy_copy->buttons[1] == 1);
            
            if (x_pressed && !b_pressed) {
                rotation = 1.0;  // 左转（逆时针）
            } else if (b_pressed && !x_pressed) {
                rotation = -1.0;  // 右转（顺时针）
            }
            // 同时按下或都不按：rotation = 0（相互抵消）
            
            target_twist.angular.z = rotation * v_angular_;
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
    rc26_telecontrol_dpad() : Node("rc26_telecontrol_dpad")
    {
        // 声明参数（不包含joy_deadzone）
        this->declare_parameter<std::string>("cmd_vel_topic", "cmd_vel_teleop");
        this->declare_parameter<double>("v_linear", 1.0);
        this->declare_parameter<double>("v_angular", 2.0);
        this->declare_parameter<double>("smoothing_alpha", 0.2);
        
        // 获取参数
        cmd_vel_topic_ = this->get_parameter("cmd_vel_topic").as_string();
        this->get_parameter("v_linear", v_linear_);
        this->get_parameter("v_angular", v_angular_);
        this->get_parameter("smoothing_alpha", smoothing_alpha_);
        
        // 约束 smoothing_alpha 在 [0.0, 1.0]
        smoothing_alpha_ = std::clamp(smoothing_alpha_, 0.0, 1.0);
        
        // QoS
        rclcpp::QoS qos_profile(10);
        qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
        qos_profile.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
        qos_profile.history(RMW_QOS_POLICY_HISTORY_KEEP_LAST);
        
        // Pub/Sub
        pub_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, qos_profile);
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", qos_profile, std::bind(&rc26_telecontrol_dpad::joy_callback, this, std::placeholders::_1));

        // 定时器 50Hz
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&rc26_telecontrol_dpad::timer_callback, this));

        // 启动日志
        RCLCPP_INFO(this->get_logger(), "rc26_telecontrol_dpad (D-Pad + Button control) ready.");
        RCLCPP_INFO(this->get_logger(),
                    "v_linear=%.2f m/s, v_angular=%.2f rad/s, smoothing_alpha=%.2f",
                    v_linear_, v_angular_, smoothing_alpha_);
        RCLCPP_INFO(this->get_logger(), "publish cmd_vel topic: %s", cmd_vel_topic_.c_str());
        RCLCPP_INFO(this->get_logger(),
                    "D-Pad: Up/Down(axes[7]) for forward/backward, Left/Right(axes[6]) for strafe");
        RCLCPP_INFO(this->get_logger(),
                    "Buttons: X(button[2]) for counter-clockwise, B(button[1]) for clockwise rotation");
    }
    
    ~rc26_telecontrol_dpad()
    {
        // 退出时发一次停止
        geometry_msgs::msg::Twist stop_msg;
        pub_->publish(stop_msg);
        RCLCPP_INFO(this->get_logger(), "rc26_telecontrol_dpad stopped.");
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<rc26_telecontrol_dpad>());
    rclcpp::shutdown();
    return 0;
}
