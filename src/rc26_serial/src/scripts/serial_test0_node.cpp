/// ros2 run rc26_decision serial_test0_node
#include <cctype>
#include <cstring>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "rc26_serial/protocol.hpp"
#include "rc26_serial/serial_driver.hpp"

namespace rc26_decision {

namespace {
std::string bytesToHex(const std::vector<uint8_t>& bytes) {
    std::ostringstream oss;
    oss << std::uppercase << std::hex << std::setfill('0');
    for (size_t i = 0; i < bytes.size(); ++i) {
        if (i > 0)
            oss << ' ';
        oss << std::setw(2) << static_cast<int>(bytes[i]);
    }
    return oss.str();
}

std::vector<uint8_t> parseHexBytes(std::string text) {
    std::vector<uint8_t> out;
    int hi = -1;
    for (char ch : text) {
        int v = -1;
        if (ch >= '0' && ch <= '9')
            v = ch - '0';
        else if (ch >= 'a' && ch <= 'f')
            v = 10 + (ch - 'a');
        else if (ch >= 'A' && ch <= 'F')
            v = 10 + (ch - 'A');
        else
            continue;

        if (hi < 0)
            hi = v;
        else {
            out.push_back(static_cast<uint8_t>((hi << 4) | v));
            hi = -1;
        }
    }
    return out;
}
}  // namespace

class SerialTestNode : public rclcpp::Node {
public:
    SerialTestNode() : Node("serial_test0_node") {
        this->declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
        this->declare_parameter<int>("baudrate", static_cast<int>(UART_BAUDRATE));
        this->declare_parameter<int>("test_cmd", static_cast<int>(CommandID::NAV_STAIR_UP));
        this->declare_parameter<std::string>("test_payload_hex", "");
        this->declare_parameter<int>("period_ms", 1000);

        const std::string port = this->get_parameter("serial_port").as_string();
        const int baudrate = this->get_parameter("baudrate").as_int();

        serial_ = std::make_unique<SerialDriver>();

        if (!serial_->open(port, baudrate)) {
            RCLCPP_FATAL(this->get_logger(), "串口打开失败: %s", serial_->lastError().c_str());
            throw std::runtime_error("串口打开失败");
        }

        RCLCPP_INFO(this->get_logger(), "串口已打开: %s @ %d", port.c_str(), baudrate);

        runTest();
    }

private:
    void runTest() {
        const int test_cmd = this->get_parameter("test_cmd").as_int();
        const std::string payload_hex = this->get_parameter("test_payload_hex").as_string();
        const int period_ms = this->get_parameter("period_ms").as_int();

        if (period_ms <= 0) {
            RCLCPP_FATAL(this->get_logger(), "参数 period_ms 必须 > 0，当前=%d", period_ms);
            throw std::runtime_error("period_ms 非法");
        }

        const uint8_t cmd = static_cast<uint8_t>(test_cmd & 0xFF);
        const std::vector<uint8_t> payload = parseHexBytes(payload_hex);

        serial_->setReceiveCallback([this](uint8_t seq, uint8_t cmd, const std::vector<uint8_t>& payload) {
            RCLCPP_INFO(this->get_logger(), "[RX_CALLBACK] seq=%u cmd=0x%02X payload_len=%zu payload=%s", seq, cmd,
                        payload.size(), bytesToHex(payload).c_str());
        });

        serial_->setDebugCallback([this](bool is_tx, const std::vector<uint8_t>& data) {
            const std::string dir = is_tx ? "TX" : "RX";
            const std::string hex = bytesToHex(data);
            RCLCPP_INFO(this->get_logger(), "[%s] size=%zu data=%s", dir.c_str(), data.size(), hex.c_str());
        });

        timer_ = this->create_wall_timer(std::chrono::milliseconds(period_ms), [this, cmd, payload]() {
            bool ok = false;
            if (cmd == static_cast<uint8_t>(CommandID::HEARTBEAT) && payload.empty()) {
                ok = serial_->sendHeartbeat();
            } else {
                ok = serial_->sendCommand(cmd, payload);
            }

            if (!ok) {
                RCLCPP_ERROR(this->get_logger(), "发送失败: %s", serial_->lastError().c_str());
            }
        });
    }

    std::unique_ptr<SerialDriver> serial_;
    rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace rc26_decision

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    try {
        auto node = std::make_shared<rc26_decision::SerialTestNode>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_FATAL(rclcpp::get_logger("serial_test_node"), "启动失败: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}
