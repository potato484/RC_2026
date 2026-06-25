// RC2026 串口通信协议 v3.0
// 通信方式：UART | 波特率：1000000 | 校验：CRC32 | 字节序：小端
#pragma once

#include <cstddef>
#include <cstdint>

namespace rc26_decision {

// ============================================================================
// 帧结构常量
// ============================================================================
constexpr uint8_t FRAME_HEAD_0 = 0xAA;
constexpr uint8_t FRAME_HEAD_1 = 0x55;
constexpr uint8_t FRAME_TAIL_0 = 0x55;
constexpr uint8_t FRAME_TAIL_1 = 0xAA;

constexpr uint32_t UART_BAUDRATE = 1000000;
constexpr uint32_t ACK_TIMEOUT_MS = 100;
constexpr uint32_t RECONNECT_INTERVAL_MS = 500;   // 重连间隔500ms
constexpr uint32_t HEARTBEAT_INTERVAL_MS = 1000;  // 心跳间隔1秒
constexpr uint8_t MAX_RETRY_VALUE = 0x09;         // 最大重发次数值
constexpr uint8_t RETRIES_PER_ROUND = 3;          // 每轮重发次数
constexpr uint8_t MAX_HEARTBEAT_FAILURES = 3;     // 心跳连续失败次数阈值
constexpr uint8_t MAX_PAYLOAD_SIZE = 32;          // payload 最大字节数
constexpr uint8_t MAX_RECONNECT_ATTEMPTS = 10;    // 最大重连尝试次数

// ============================================================================
// 下行指令 ID (上位机 -> MCU)
// ============================================================================
enum class CommandID : uint8_t {
    STOP = 0x00,              // 急停指令
    GRAB_TIP = 0x01,          // 抓取端头
    ASSEMBLE_WEAPON = 0x02,   // 组装兵器
    GRAB_KFS = 0x07,          // 梅林区夹取 KFS
    PLACE_KFS_GRID = 0x0B,    // 对抗区放置 KFS 到九宫格
    HEARTBEAT = 0x0D,             // 心跳查询请求
    FRONT_PUSHROD_EXTEND = 0x0E,  // 前推杆伸展
    FRONT_PUSHROD_RETRACT = 0x0F, // 前推杆收缩
    REAR_PUSHROD_EXTEND = 0x10,   // 后推杆伸展
    REAR_PUSHROD_RETRACT = 0x11,  // 后推杆收缩
    POSE_TARGET = 0x1F,       // 目标速度 (vx, vy, wz) - MCU速度闭环
};

// ============================================================================
// 上行反馈 ID (MCU -> 上位机)
// ============================================================================
enum class FeedbackID : uint8_t {
    ACK = 0x00,                    // 确认收到指令
    NACK = 0x01,                   // 执行动作失败，需要继续发送动作指令
    GRAB_TIP_DONE = 0x02,          // 夹取端头完成
    ASSEMBLE_DONE = 0x03,          // 组装兵器完成
    GRAB_KFS_DONE = 0x0C,          // 夹取 KFS 完成
    PLACE_KFS_GRID_DONE = 0x0E,    // 九宫格放置完成
    HEARTBEAT_ACK = 0x10,          // 心跳响应
    FRONT_LASER_HEIGHT_JUMP = 0x17,   // 前轮附近第一个激光测距模块检测到车体高度突变
    REAR_LASER_HEIGHT_JUMP = 0x18,    // 后轮附近激光测距模块检测到车体高度突变
    FRONT_LIMIT_SWITCH_TRIGGERED = 0x19, // 武馆前方限位开关触发，payload v1 为空或忽略
    FRONT_SECOND_LASER_HEIGHT_JUMP = 0x1A, // 前轮附近第二个激光测距模块检测到车体高度突变
    ODOM_DATA = 0x20,                 // 麦克纳姆轮速反馈：<v_fl,v_rl,v_rr,v_fr>，单位: m/s
};

// ============================================================================
// 决策阶段定义
// ============================================================================
enum class Phase : uint8_t {
    MC_TAKE,      // 武馆取端头
    MC_ASSEMBLE,  // 武馆组装兵器
    MF_ENTRY,     // 梅林入口观察
    MF_CORE,      // 梅林内部决策
    MF_LEAVE,     // 离开梅林
    GOTO_COMBAT,  // 前往对抗区
    COMBAT,       // 对抗阶段
};

// ============================================================================
// 梅林格子状态
// ============================================================================
enum class GridState : uint8_t {
    UNKNOWN,     // 未知
    EMPTY,       // 空
    AUTO_KFS,    // 自动 KFS
    MANUAL_KFS,  // 手动 KFS
    FAKE_KFS,    // 假 KFS
};

// ============================================================================
// 帧结构体 (1字节对齐)
// ============================================================================
#pragma pack(push, 1)

struct FrameHeader {
    uint8_t head[2];  // 0xAA, 0x55
    uint8_t seq;      // 序列号
    uint8_t len;      // cmd(1B) + payload(NB)，1字节
    uint8_t retry;    // 可靠命令的重发编号；连续命令固定为 0x00
    uint8_t cmd;      // 命令 ID
};

#pragma pack(pop)

// ============================================================================
// 导航点坐标 (Map 坐标系)
// ============================================================================
namespace waypoint {

// 红方
namespace red {
constexpr double GRAB_TIP_X = -0.7;
constexpr double GRAB_TIP_Y = 0.6;
constexpr double ASSEMBLE_X = -0.5;
constexpr double ASSEMBLE_Y = 0.2;
constexpr double MF_ENTRY_X = 1.6;
constexpr double MF_ENTRY_Y = 2.0;
constexpr double ORIGIN_X = -1.4;
constexpr double ORIGIN_Y = -0.3;
}  // namespace red

// 蓝方
namespace blue {
constexpr double GRAB_TIP_X = 0.7;
constexpr double GRAB_TIP_Y = 0.6;
constexpr double ASSEMBLE_X = 0.5;
constexpr double ASSEMBLE_Y = 0.2;
constexpr double MF_ENTRY_X = -1.6;
constexpr double MF_ENTRY_Y = 2.0;
constexpr double ORIGIN_X = 1.4;
constexpr double ORIGIN_Y = -0.3;
}  // namespace blue

}  // namespace waypoint

// ============================================================================
// CRC32 计算 (MPEG-2 模型)
// ============================================================================
inline uint32_t crc32_mpeg2_calculate(const uint8_t* data, size_t len) {
    uint32_t crc = 0xFFFFFFFF;
    for (size_t i = 0; i < len; ++i) {
        crc ^= (uint32_t)data[i] << 24;  // 将数据移到高位进行异或
        for (uint8_t j = 0; j < 8; ++j) {
            if (crc & 0x80000000)
                crc = (crc << 1) ^ 0x04C11DB7;  // 多项式 0x04C11DB7
            else
                crc <<= 1;
        }
    }
    return crc;  // 无需最终异或
}

}  // namespace rc26_decision

namespace rc26_serial {

using ::rc26_decision::CommandID;
using ::rc26_decision::FeedbackID;

}  // namespace rc26_serial
