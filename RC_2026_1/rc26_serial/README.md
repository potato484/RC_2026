# rc26_serial

RC2026 串口通信驱动库，用于上位机与下位机 MCU 之间的通信。

## 功能

- UART 串口通信
- CRC32 校验 (MPEG-2 模型)
- 自动重发机制
- ACK/NACK 确认机制
- 心跳检测与自动重连
- 异步接收回调

## 协议规格

| 项目 | 规格 |
|------|------|
| 通信方式 | UART |
| 波特率 | 115200 |
| 数据位 | 8 |
| 校验位 | 无 |
| 停止位 | 1 |
| 校验算法 | CRC32 (MPEG-2) |
| 字节序 | 小端 |

## 帧结构

```
┌──────┬──────┬─────┬─────┬───────┬─────┬─────────┬───────┬──────┬──────┐
│ HEAD │ HEAD │ SEQ │ LEN │ RETRY │ CMD │ PAYLOAD │ CRC32 │ TAIL │ TAIL │
│ 0xAA │ 0x55 │ 1B  │ 1B  │  1B   │ 1B  │   NB    │  4B   │ 0x55 │ 0xAA │
└──────┴──────┴─────┴─────┴───────┴─────┴─────────┴───────┴──────┴──────┘
```

| 字段 | 长度 | 说明 |
|------|------|------|
| HEAD | 2B | 帧头 `0xAA 0x55` |
| SEQ | 1B | 序列号 (0-255 循环) |
| LEN | 1B | CMD + PAYLOAD 长度 |
| RETRY | 1B | 重发次数 (0x00/0x03/0x06/0x09) |
| CMD | 1B | 命令 ID |
| PAYLOAD | NB | 数据载荷 |
| CRC32 | 4B | CRC32 校验值 |
| TAIL | 2B | 帧尾 `0x55 0xAA` |

## 下行指令 (上位机 -> MCU)

| ID | 命令 | 说明 |
|----|------|------|
| 0x00 | NAV_NORMAL | 非梅林区导航 |
| 0x01 | NAV_STAIR_UP | 梅林区上阶梯导航 |
| 0x02 | NAV_STAIR_DOWN | 梅林区下阶梯导航 |
| 0x03 | STOP | 急停指令 |
| 0x04 | GRAB_TIP | 抓取端头 |
| 0x05 | ASSEMBLE_WEAPON | 组装兵器 |
| 0x06 | ROTATE_POS_90 | 旋转 +90° |
| 0x07 | ROTATE_NEG_90 | 旋转 -90° |
| 0x08 | ROTATE_POS_180 | 旋转 +180° |
| 0x09 | ROTATE_NEG_180 | 旋转 -180° |
| 0x0A | GRAB_KFS | 梅林区夹取 KFS |
| 0x0B | MECH_UP_MERLIN | 梅林区机构抬升 |
| 0x0C | MECH_DOWN_MERLIN | 梅林区机构下降 |
| 0x0D | MECH_UP_DUEL | 对抗区机构抬升 |
| 0x0E | PLACE_KFS_GRID | 放置 KFS 到九宫格 |
| 0x0F | PLACE_KFS_GROUND | 放置 KFS 到地面 |
| 0x10 | HEARTBEAT | 心跳查询请求 |
| 0x20 | POSE_DATA | 实时底盘位姿 |

### 位姿数据载荷 (POSE_DATA)

```cpp
struct PosePayload {
    float vx;     // 线速度 X
    float vy;     // 线速度 Y
    float wx;     // 世界坐标 X
    float wy;     // 世界坐标 Y
    float wz;     // 世界坐标 Z
    float roll;   // 横滚角
    float pitch;  // 俯仰角
    float yaw;    // 偏航角
};  // 32 字节
```

## 上行反馈 (MCU -> 上位机)

| ID | 反馈 | 说明 |
|----|------|------|
| 0x00 | ACK | 确认收到指令 |
| 0x01 | NACK | 执行动作失败 |
| 0x02 | GRAB_TIP_DONE | 夹取端头完成 |
| 0x03 | ASSEMBLE_DONE | 组装兵器完成 |
| 0x04 | CLIMBING_SLOPE | 正在爬坡 |
| 0x05 | SLOPE_DONE | 爬坡完成 |
| 0x06 | ROTATE_POS_90_DONE | 旋转 +90° 完成 |
| 0x07 | ROTATE_NEG_90_DONE | 旋转 -90° 完成 |
| 0x08 | ROTATE_POS_180_DONE | 旋转 +180° 完成 |
| 0x09 | ROTATE_NEG_180_DONE | 旋转 -180° 完成 |
| 0x0A | MECH_UP_MERLIN_DONE | 梅林区机构抬升完成 |
| 0x0B | MECH_DOWN_MERLIN_DONE | 梅林区机构下降完成 |
| 0x0C | GRAB_KFS_DONE | 夹取 KFS 完成 |
| 0x0D | MECH_UP_DUEL_DONE | 对抗区机构抬升完成 |
| 0x0E | PLACE_KFS_GRID_DONE | 九宫格放置完成 |
| 0x0F | PLACE_KFS_GROUND_DONE | 地面放置完成 |
| 0x10 | HEARTBEAT_ACK | 心跳响应 |
| 0x11 | STAIR_CLIMB_DONE | 上阶梯完成 |
| 0x12 | STAIR_DESCEND_DONE | 下阶梯完成 |
| 0x20 | ODOM_DATA | 轮式里程计数据 |
| 0xFE | ACTION_FAIL | 动作执行失败 |
| 0xFF | ERROR | 系统致命异常 |

## 文件结构

```
rc26_serial/
├── include/rc26_serial/
│   ├── protocol.hpp       # 协议定义 (帧结构、命令ID、CRC32)
│   └── serial_driver.hpp  # 串口驱动头文件
├── src/
│   └── serial_driver.cpp  # 串口驱动实现
├── CMakeLists.txt
└── package.xml
```

## API 使用

### 基本使用

```cpp
#include "rc26_serial/serial_driver.hpp"

rc26_decision::SerialDriver serial;

// 打开串口
if (!serial.open("/dev/ttyUSB0", 115200)) {
    std::cerr << "打开串口失败: " << serial.lastError() << std::endl;
    return;
}

// 发送命令 (使用枚举)
serial.sendCommand(rc26_decision::CommandID::GRAB_TIP);

// 发送命令 (使用原始 ID)
serial.sendCommand(0x04);

// 发送急停
serial.sendStop();

// 发送心跳
serial.sendHeartbeat();

// 关闭串口
serial.close();
```

### 发送位姿数据

```cpp
// 发送实时位姿
serial.sendPose(
    vx, vy,           // 线速度
    wx, wy, wz,       // 世界坐标
    roll, pitch, yaw  // 姿态角
);
```

### 设置回调

```cpp
// 接收回调
serial.setReceiveCallback([](uint8_t seq, uint8_t cmd, const std::vector<uint8_t>& payload) {
    switchst<rc26_decision::FeedbackID>(cmd)) {
        case rc26_decision::FeedbackID::GRAB_TIP_DONE:
            std::cout << "夹取端头完成" << std::endl;
            break;
        case rc26_decision::FeedbackID::ACK:
            std::cout << "收到 ACK" << std::endl;
            break;
        // ...
    }
});

// 心跳失败回调
serial.setHeartbeatFailureCallback([]() {
    std::cerr << "心跳失败!" << std::endl;
});

// 重连开始回调
serial.setReconnectStartCallback([]() {
    std::cout << "开始重连..." << std::endl;
});

// 重连成功回调
serial.setReconnectCallback([]() {
    std::cout << "重连成功!" << std::endl;
});

// 调试回调 (查看原始数据)
serial.setDebugCallback([](bool is_tx, const std::vector<uint8_t>& data) {
    std::cout << (is_tx ? "TX: " : "RX: ");
    for (auto b : data) printf("%02X ", b);
    std::cout << std::endl;
});
```

## 协议常量

```cpp
constexpr uint8_t FRAME_HEAD_0 = 0xAA;
constexpr uint8_t FRAME_HEAD_1 = 0x55;
constexpr uint8_t FRAME_TAIL_0 = 0x55;
constexpr uint8_t FRAME_TAIL_1 = 0xAA;

constexpr uint32_t UART_BAUDRATE = 115200;
constexpr uint32_t ACK_TIMEOUT_MS = 100;
constexpr uint32_t RECONNECT_INTERVAL_MS = 500;
constexpr uint32_t HEARTBEAT_INTERVAL_MS = 1000;
constexpr uint8_t MAX_RETRY_VALUE = 0x09;
constexpr uint8_t RETRIES_PER_ROUND = 3;
constexpr uint8_t MAX_HEARTBEAT_FAILURES = 3;
```

## CRC32 校验

使用 MPEG-2 模型的 CRC32 算法：
- 多项式: 0x04C11DB7
- 初始值: 0xFFFFFFFF
- 无最终异或

```cpp
uint32_t crc = rc26_decision::crc32_mpeg2_calculate(data, len);
```

## 依赖

- rclcpp (仅用于日志，可选)

## 编译

```bash
colcon build --packages-select rc26_serial
```
