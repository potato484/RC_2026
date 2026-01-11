// 单轮CAN通信最小测试单元
// 仅监听单个轮子(FRONT_LEFT, CAN ID 0x201)的电机反馈数据
#include <rclcpp/rclcpp.hpp>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <cstring>
#include <thread>

class SingleWheelTestNode : public rclcpp::Node
{
public:
    SingleWheelTestNode() : Node("single_wheel_test_node")
    {
        this->declare_parameter("can_interface", "can0");
        this->declare_parameter("motor_id", 1);  // 1=FL, 2=RL, 3=RR, 4=FR
        this->declare_parameter("print_rate_hz", 10);

        can_interface_ = this->get_parameter("can_interface").as_string();
        motor_id_ = this->get_parameter("motor_id").as_int();
        int print_rate = this->get_parameter("print_rate_hz").as_int();

        target_can_id_ = 0x200 + motor_id_;

        if (!initCan())
        {
            RCLCPP_ERROR(this->get_logger(), "CAN初始化失败");
            return;
        }

        running_ = true;
        can_thread_ = std::thread(&SingleWheelTestNode::canThreadFunc, this);

        auto period = std::chrono::milliseconds(1000 / print_rate);
        print_timer_ = this->create_wall_timer(period, std::bind(&SingleWheelTestNode::printStatus, this));

        RCLCPP_INFO(this->get_logger(), "单轮测试启动: interface=%s, motor_id=%d, can_id=0x%X",
                    can_interface_.c_str(), motor_id_, target_can_id_);
    }

    ~SingleWheelTestNode()
    {
        running_ = false;
        if (can_thread_.joinable())
        {
            can_thread_.join();
        }
        if (can_socket_ >= 0)
        {
            close(can_socket_);
        }
    }

private:
    bool initCan()
    {
        can_socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (can_socket_ < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "创建CAN套接字失败: %s", strerror(errno));
            return false;
        }

        struct ifreq ifr;
        std::strncpy(ifr.ifr_name, can_interface_.c_str(), IFNAMSIZ - 1);
        ifr.ifr_name[IFNAMSIZ - 1] = '\0';

        if (ioctl(can_socket_, SIOCGIFINDEX, &ifr) < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "获取CAN接口索引失败: %s", strerror(errno));
            close(can_socket_);
            can_socket_ = -1;
            return false;
        }

        struct sockaddr_can addr{};
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;

        if (bind(can_socket_, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "绑定CAN套接字失败: %s", strerror(errno));
            close(can_socket_);
            can_socket_ = -1;
            return false;
        }

        // 仅过滤目标电机的CAN ID
        struct can_filter filter;
        filter.can_id = target_can_id_;
        filter.can_mask = CAN_SFF_MASK;
        setsockopt(can_socket_, SOL_CAN_RAW, CAN_RAW_FILTER, &filter, sizeof(filter));

        struct timeval tv;
        tv.tv_sec = 0;
        tv.tv_usec = 100000;
        setsockopt(can_socket_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

        return true;
    }

    void canThreadFunc()
    {
        struct can_frame frame;
        while (running_)
        {
            ssize_t nbytes = read(can_socket_, &frame, sizeof(frame));
            if (nbytes < 0)
            {
                if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)
                {
                    continue;
                }
                RCLCPP_WARN(this->get_logger(), "CAN读取错误: %s", strerror(errno));
                continue;
            }

            if (nbytes == sizeof(frame) && frame.can_id == target_can_id_ && frame.can_dlc >= 8)
            {
                std::lock_guard<std::mutex> lock(data_mutex_);
                angle_raw_ = (static_cast<uint16_t>(frame.data[0]) << 8) | frame.data[1];
                rpm_ = static_cast<int16_t>((static_cast<uint16_t>(frame.data[2]) << 8) | frame.data[3]);
                current_ = static_cast<int16_t>((static_cast<uint16_t>(frame.data[4]) << 8) | frame.data[5]);
                temperature_ = frame.data[6];
                recv_count_++;
                data_valid_ = true;
            }
        }
    }

    void printStatus()
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        if (data_valid_)
        {
            double angle_deg = static_cast<double>(angle_raw_) * 360.0 / 8192.0;
            RCLCPP_INFO(this->get_logger(),
                        "[Motor %d] RPM: %6d | Angle: %6.1f deg | Current: %5d | Temp: %3d C | Count: %lu",
                        motor_id_, rpm_, angle_deg, current_, temperature_, recv_count_);
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "[Motor %d] 无数据 (CAN ID 0x%X)", motor_id_, target_can_id_);
        }
    }

    std::string can_interface_;
    int motor_id_;
    uint32_t target_can_id_;
    int can_socket_ = -1;

    std::atomic<bool> running_{false};
    std::thread can_thread_;
    rclcpp::TimerBase::SharedPtr print_timer_;

    std::mutex data_mutex_;
    uint16_t angle_raw_ = 0;
    int16_t rpm_ = 0;
    int16_t current_ = 0;
    uint8_t temperature_ = 0;
    uint64_t recv_count_ = 0;
    bool data_valid_ = false;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SingleWheelTestNode>());
    rclcpp::shutdown();
    return 0;
}
