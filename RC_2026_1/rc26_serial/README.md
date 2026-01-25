# rc26_serial

## 1. 模块简介 (Introduction)
`rc26_serial` 实现了上位机（PC/Jetson）与下位机（MCU/STM32）之间的高性能串口通信。它基于自定义的帧协议，提供可靠的数据传输、丢包重发、心跳保活及自动重连机制。

本模块是机器人控制系统的通信核心，负责将上位机的控制指令（如速度、动作命令）下发给 MCU，同时接收 MCU 反馈的里程计数据和动作执行结果。

## 2. 核心功能 (Core Features)
*   **可靠传输协议**：基于 `0xAA 0x55` 帧头和 CRC32 校验，支持 ACK 确认与超时重传 (ARQ)。
*   **双向通信**：
    *   **TX (下行)**: 发送控制指令（如速度闭环控制、机械臂动作、导航目标点）。
    *   **RX (上行)**: 接收里程计反馈、动作执行结果、传感器数据。
*   **异常处理**：
    *   **断线重连**：检测串口设备丢失或心跳超时后，自动尝试重新打开串口。
    *   **线程安全**：发送与接收运行在独立线程，通过互斥锁保护共享资源。
*   **心跳保活**：定期发送心跳包，检测通信链路健康状态。

## 3. 通信协议 (Protocol v3.0)

### 3.1 物理层
| 参数 | 值 |
| :--- | :--- |
| **波特率** | 1,000,000 (1Mbps) |
| **数据位** | 8 |
| **校验位** | None |
| **停止位** | 1 |
| **字节序** | 小端 (Little-Endian) |

### 3.2 帧结构
所有数据按**小端 (Little-Endian)** 字节序传输。

| 字段 | 长度 (Byte) | 值/说明 |
| :--- | :--- | :--- |
| HEAD_0 | 1 | `0xAA` |
| HEAD_1 | 1 | `0x55` |
| SEQ | 1 | 序列号 (0-255, 循环) |
| LEN | 1 | CMD(1) + Payload(N) 的总长度 |
| RETRY | 1 | 重发计数 (0x00, 0x03, 0x06, 0x09) |
| CMD | 1 | 命令 ID |
| PAYLOAD | N | 数据载荷 (Max 32 Bytes) |
| CRC32 | 4 | 整个帧的 MPEG-2 CRC32 校验 |
| TAIL_0 | 1 | `0x55` |
| TAIL_1 | 1 | `0xAA` |

**帧结构示意图**:
```
┌──────┬──────┬─────┬─────┬───────┬─────┬─────────┬───────┬──────┬──────┐
│ 0xAA │ 0x55 │ SEQ │ LEN │ RETRY │ CMD │ PAYLOAD │ CRC32 │ 0x55 │ 0xAA │
└──────┴──────┴─────┴─────┴───────┴─────┴─────────┴───────┴──────┴──────┘
   1B     1B    1B    1B     1B     1B    0-32B      4B     1B     1B
```

### 3.3 下行指令 ID (上位机 → MCU)

| ID | 名称 | 说明 | Payload |
| :--- | :--- | :--- | :--- |
| `0x00` | NAV_NORMAL | 非梅林区导航 | - |
| `0x01` | NAV_STAIR_UP | 梅林区上阶梯导航 | - |
| `0x02` | NAV_STAIR_DOWN | 梅林区下阶梯导航 | - |
| `0x03` | STOP | 急停指令 | - |
| `0x04` | GRAB_TIP | 抓取端头 | - |
| `0x05` | ASSEMBLE_WEAPON | 组装兵器 | - |
| `0x06` | ROTATE_POS_90 | 旋转 +90° | - |
| `0x07` | ROTATE_NEG_90 | 旋转 -90° | - |
| `0x08` | ROTATE_POS_180 | 旋转 +180° | - |
| `0x09` | ROTATE_NEG_180 | 旋转 -180° | - |
| `0x0A` | GRAB_KFS | 梅林区夹取 KFS | - |
| `0x0B` | MECH_UP_MERLIN | 梅林区机构抬升 | - |
| `0x0C` | MECH_DOWN_MERLIN | 梅林区机构下降 | - |
| `0x0D` | MECH_UP_DUEL | 对抗区机构抬升 | - |
| `0x0E` | PLACE_KFS_GRID | 对抗区放置 KFS 到九宫格 | - |
| `0x0F` | PLACE_KFS_GROUND | 对抗区放置 KFS 到地面 | - |
| `0x10` | HEARTBEAT | 心跳查询请求 | - |
| `0x21` | POSE_FEEDBACK | 反馈速度 (vx, vy, wz) | 12B (3×float) |
| `0x22` | POSE_TARGET | 目标速度 (vx, vy, wz) | 12B (3×float) |

### 3.4 上行反馈 ID (MCU → 上位机)

| ID | 名称 | 说明 | Payload |
| :--- | :--- | :--- | :--- |
| `0x00` | ACK | 确认收到指令 | - |
| `0x01` | NACK | 执行动作失败，需要继续发送动作指令 | - |
| `0x02` | GRAB_TIP_DONE | 夹取端头完成 | - |
| `0x03` | ASSEMBLE_DONE | 组装兵器完成 | - |
| `0x04` | CLIMBING_SLOPE | 正在爬坡 | - |
| `0x05` | SLOPE_DONE | 爬坡完成 | - |
| `0x06` | ROTATE_POS_90_DONE | 旋转 +90° 完成 | - |
| `0x07` | ROTATE_NEG_90_DONE | 旋转 -90° 完成 | - |
| `0x08` | ROTATE_POS_180_DONE | 旋转 +180° 完成 | - |
| `0x09` | ROTATE_NEG_180_DONE | 旋转 -180° 完成 | - |
| `0x0A` | MECH_UP_MERLIN_DONE | 梅林区机构抬升完成 | - |
| `0x0B` | MECH_DOWN_MERLIN_DONE | 梅林区机构下降完成 | - |
| `0x0C` | GRAB_KFS_DONE | 夹取 KFS 完成 | - |
| `0x0D` | MECH_UP_DUEL_DONE | 对抗区机构抬升完成 | - |
| `0x0E` | PLACE_KFS_GRID_DONE | 九宫格放置完成 | - |
| `0x0F` | PLACE_KFS_GROUND_DONE | 地面放置完成 | - |
| `0x10` | HEARTBEAT_ACK | 心跳响应 | - |
| `0x11` | STAIR_CLIMB_DONE | 上阶梯完成 | - |
| `0x12` | STAIR_DESCEND_DONE | 下阶梯完成 | - |
| `0x20` | ODOM_DATA | 轮式里程计数据 | 16B (4×float: v_fl, v_rl, v_rr, v_fr) |
| `0xFE` | ACTION_FAIL | 动作执行失败 | - |
| `0xFF` | ERROR | 系统致命异常 | - |

### 3.5 协议常量
```cpp
constexpr uint32_t UART_BAUDRATE = 1000000;
constexpr uint32_t ACK_TIMEOUT_MS = 100;           // ACK 等待超时
constexpr uint32_t RECONNECT_INTERVAL_MS = 500;    // 重连间隔
constexpr uint32_t HEARTBEAT_INTERVAL_MS = 1000;   // 心跳间隔
constexpr uint8_t MAX_RETRY_VALUE = 0x09;          // 最大重发次数值
constexpr uint8_t RETRIES_PER_ROUND = 3;           // 每轮重发次数
constexpr uint8_t MAX_HEARTBEAT_FAILURES = 3;      // 心跳连续失败次数阈值
constexpr uint8_t MAX_PAYLOAD_SIZE = 32;           // payload 最大字节数
constexpr uint8_t MAX_RECONNECT_ATTEMPTS = 10;     // 最大重连尝试次数
```

## 4. 代码实现细节

### 4.1 SerialDriver 类
*   封装了 `termios` 串口操作（Linux）。
*   **发送线程**：`sendCommand` 接口将数据打包并写入串口。如果是需要 ACK 的指令，会阻塞等待 `ack_cv_` 或超时。
*   **接收线程**：`recvThreadFunc` 循环读取串口数据到环形缓冲区，根据帧头帧尾截取完整数据包，并通过 CRC32 校验。
*   **重发机制**：若发送后 `ACK_TIMEOUT_MS` (100ms) 内未收到对应 SEQ 的 ACK 包，且重试次数未达上限，则自动重发。

### 4.2 心跳与重连
*   **心跳**：上位机定期发送 `HEARTBEAT` (0x10)，下位机回复 `HEARTBEAT_ACK`。
*   **故障检测**：连续 `MAX_HEARTBEAT_FAILURES` 次心跳失败，或 `write` 操作返回错误，触发重连流程。
*   **重连流程**：关闭当前 fd → 等待 `RECONNECT_INTERVAL_MS` → 尝试 `open()` → 恢复数据流。

### 4.3 CRC32 校验 (MPEG-2)
```cpp
inline uint32_t crc32_mpeg2_calculate(const uint8_t* data, size_t len) {
    uint32_t crc = 0xFFFFFFFF;
    for (size_t i = 0; i < len; ++i) {
        crc ^= (uint32_t)data[i] << 24;
        for (uint8_t j = 0; j < 8; ++j) {
            if (crc & 0x80000000)
                crc = (crc << 1) ^ 0x04C11DB7;
            else
                crc <<= 1;
        }
    }
    return crc;
}
```

## 5. 接口说明 (Interface Description)

### 5.1 C++ API

**初始化与连接**:
```cpp
#include "rc26_serial/serial_driver.hpp"

rc26_decision::SerialDriver driver;

// 打开串口
if (!driver.open("/dev/ttyUSB0", 1000000)) {
    std::cerr << "Failed to open serial: " << driver.lastError() << std::endl;
}

// 检查连接状态
if (driver.isOpen()) {
    // 串口已连接
}

// 关闭串口
driver.close();
```

**设置回调**:
```cpp
// 接收数据回调
driver.setReceiveCallback([](uint8_t seq, uint8_t cmd, const std::vector<uint8_t>& payload) {
    using namespace rc26_decision;
    auto feedback = static_cast<FeedbackID>(cmd);

    switch (feedback) {
        case FeedbackID::ODOM_DATA:
            // 解析里程计数据 (4 个 float)
            if (payload.size() >= 16) {
                float v_fl, v_rl, v_rr, v_fr;
                std::memcpy(&v_fl, payload.data(), 4);
                std::memcpy(&v_rl, payload.data() + 4, 4);
                std::memcpy(&v_rr, payload.data() + 8, 4);
                std::memcpy(&v_fr, payload.data() + 12, 4);
            }
            break;
        case FeedbackID::GRAB_TIP_DONE:
            // 抓取端头完成
            break;
        case FeedbackID::ACTION_FAIL:
            // 动作执行失败
            break;
        default:
            break;
    }
});

// 心跳失败回调
driver.setHeartbeatFailureCallback([]() {
    std::cerr << "Heartbeat failure detected!" << std::endl;
});

// 重连成功回调
driver.setReconnectCallback([]() {
    std::cout << "Serial reconnected successfully" << std::endl;
});

// 重连开始回调
driver.setReconnectStartCallback([]() {
    std::cout << "Starting reconnection..." << std::endl;
});

// 重连失败回调
driver.setReconnectFailedCallback([]() {
    std::cerr << "Reconnection failed!" << std::endl;
});

// 调试回调 (可选)
driver.setDebugCallback([](bool is_tx, const std::vector<uint8_t>& data) {
    std::cout << (is_tx ? "TX: " : "RX: ");
    for (auto b : data) printf("%02X ", b);
    std::cout << std::endl;
});
```

**发送指令**:
```cpp
using namespace rc26_decision;

// 发送简单指令
driver.sendCommand(CommandID::STOP);
driver.sendCommand(CommandID::GRAB_TIP);
driver.sendCommand(CommandID::NAV_NORMAL);

// 发送速度指令 (vx, vy, wz)
driver.sendPose(CommandID::POSE_TARGET, 0.5f, 0.0f, 0.1f);

// 发送心跳
driver.sendHeartbeat();

// 发送带自定义 payload 的指令
std::vector<uint8_t> payload = {0x01, 0x02, 0x03};
driver.sendCommand(0x50, payload);
```

### 5.2 回调类型定义
```cpp
using ReceiveCallback = std::function<void(uint8_t seq, uint8_t cmd, const std::vector<uint8_t>& payload)>;
using HeartbeatFailureCallback = std::function<void()>;
using ReconnectCallback = std::function<void()>;
using ReconnectStartCallback = std::function<void()>;
using ReconnectFailedCallback = std::function<void()>;
using DebugCallback = std::function<void(bool is_tx, const std::vector<uint8_t>& data)>;
```

## 6. 启动示例 (Usage)

### 6.1 完整使用示例
```cpp
#include "rc26_serial/serial_driver.hpp"
#include <iostream>
#include <thread>
#include <chrono>

int main() {
    using namespace rc26_decision;

    SerialDriver driver;

    // 设置回调
    driver.setReceiveCallback([](uint8_t seq, uint8_t cmd, const std::vector<uint8_t>& payload) {
        std::cout << "Received: SEQ=" << (int)seq << " CMD=0x" << std::hex << (int)cmd << std::endl;
    });

    driver.setHeartbeatFailureCallback([]() {
        std::cerr << "Heartbeat failure!" << std::endl;
    });

    driver.setReconnectCallback([]() {
        std::cout << "Reconnected!" << std::endl;
    });

    // 打开串口
    if (!driver.open("/dev/ttyUSB0")) {
        std::cerr << "Failed: " << driver.lastError() << std::endl;
        return 1;
    }

    std::cout << "Serial opened successfully" << std::endl;

    // 发送测试指令
    driver.sendCommand(CommandID::NAV_NORMAL);

    // 主循环
    while (driver.isOpen()) {
        // 定期发送心跳
        driver.sendHeartbeat();
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    driver.close();
    return 0;
}
```

### 6.2 测试节点
```bash
# 编译后运行测试节点
ros2 run rc26_serial serial_test0_node
ros2 run rc26_serial serial_test1_node
```

## 7. 目录结构 (Directory Structure)
```
rc26_serial/
├── include/rc26_serial/
│   ├── serial_driver.hpp    # SerialDriver 类定义
│   └── protocol.hpp         # 协议常量、命令 ID、帧结构定义
├── src/
│   ├── serial_driver.cpp    # SerialDriver 实现
│   └── scripts/
│       ├── serial_test0_node.cpp  # 测试节点 0
│       └── serial_test1_node.cpp  # 测试节点 1
├── package.xml              # ROS 2 包描述
├── CMakeLists.txt           # 构建配置
└── README.md                # 本文档
```

## 8. 依赖项 (Dependencies)
*   `rclcpp`: ROS 2 C++ 客户端库 (可选，用于测试节点)
*   Linux `termios`: 串口操作 API

## 9. 故障排查 (Troubleshooting)

### 9.1 串口无法打开
```bash
# 检查串口设备是否存在
ls -la /dev/ttyUSB*

# 检查权限
sudo chmod 666 /dev/ttyUSB0

# 或将用户添加到 dialout 组
sudo usermod -aG dialout $USER
# 重新登录后生效
```

### 9.2 通信不稳定
*   检查波特率是否匹配 (1000000)
*   检查 USB 线缆质量
*   检查 MCU 端协议实现是否正确
*   使用 `setDebugCallback` 打印收发数据进行调试

### 9.3 心跳超时
*   检查 MCU 是否正确响应 `HEARTBEAT_ACK`
*   检查串口是否被其他程序占用
*   检查 USB 转串口芯片驱动是否正常

## 10. 导航点坐标参考 (Waypoints)
协议中预定义了比赛场地的关键导航点坐标：

**红方 (Red)**:
| 点位 | X (m) | Y (m) |
| :--- | :--- | :--- |
| GRAB_TIP | -0.7 | 0.6 |
| ASSEMBLE | -0.5 | 0.2 |
| MF_ENTRY | 1.6 | 2.0 |
| ORIGIN | -1.4 | -0.3 |

**蓝方 (Blue)**:
| 点位 | X (m) | Y (m) |
| :--- | :--- | :--- |
| GRAB_TIP | 0.7 | 0.6 |
| ASSEMBLE | 0.5 | 0.2 |
| MF_ENTRY | -1.6 | 2.0 |
| ORIGIN | 1.4 | -0.3 |
