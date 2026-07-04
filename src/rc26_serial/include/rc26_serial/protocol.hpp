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
constexpr uint8_t PLANAR_ARM_ERROR_PAYLOAD_SIZE = 2; // 0xFE 机械臂错误反馈 payload 字节数

// ============================================================================
// 下行指令 ID (上位机 -> MCU)
// ============================================================================
enum class CommandID : uint8_t {
    STOP = 0x00,                  // 急停指令
    GRAB_TIP = 0x01,              // 抓取端头
    GRAB_KFS_DOWN = 0x02,         // 下台阶/下降方向 KFS 夹取
    GRAB_KFS_UP = 0x03,           // 上台阶/抬升方向 KFS 夹取
    ARM_RAISE = 0x04,             // 机械臂底座抬起
    ARM_LOWER = 0x05,             // 机械臂底座下降
    PLACE_KFS_GRID = 0x06,        // 对抗区放置 KFS 到九宫格
    HEARTBEAT = 0x07,             // 心跳查询请求
    FRONT_PUSHROD_EXTEND = 0x08,  // 前推杆伸展
    FRONT_PUSHROD_RETRACT = 0x09, // 前推杆收缩
    REAR_PUSHROD_EXTEND = 0x0A,   // 后推杆伸展
    REAR_PUSHROD_RETRACT = 0x0B,  // 后推杆收缩
    POSE_TARGET = 0x0C,           // 目标速度 (vx, vy, wz) - MCU速度闭环
    ARM_HIGH_RAISE = 0x0D,        // 梅林预选赛入口专属机械臂底座高抬升
    ARM_SECOND_LOWER = 0x0E,      // KFS 向下夹取前第二节机械臂彻底放下
    ENTRY_GRAB_KFS_UP = 0x0F,     // 梅林预选赛入口高侧 KFS 夹取
    COMPETITION_START = 0x10,     // 比赛开始通知
    SECOND_PRESELECTION_START = 0x11,          // 第二预选赛开始通知
    SECOND_PRESELECTION_ARM_HIGH_RAISE = 0x12, // 第二预选赛机械臂底座高抬升
    SECOND_PRESELECTION_PLACE_KFS = 0x13,      // 第二预选赛放置 KFS 到中层九宫格
    SECOND_PRESELECTION_ARM_LOWER = 0x14,      // 第二预选赛视觉对齐后机械臂彻底放下
};

// ============================================================================
// 上行反馈 ID (MCU -> 上位机)
// ============================================================================
enum class FeedbackID : uint8_t {
    ACK = 0x00,                         // 确认收到指令
    HEARTBEAT_ACK = 0x01,               // 心跳响应
    ARM_RAISE_DONE = 0x02,              // 机械臂抬起完成
    ARM_LOWER_DONE = 0x03,              // 机械臂下降完成
    FRONT_LASER_HEIGHT_JUMP = 0x04,     // 前轮附近第一个激光测距模块检测到车体高度突变
    REAR_LASER_HEIGHT_JUMP = 0x05,      // 后轮附近激光测距模块检测到车体高度突变
    FRONT_LIMIT_SWITCH_TRIGGERED = 0x06, // 武馆前方限位开关触发，payload v1 为空或忽略
    FRONT_SECOND_LASER_HEIGHT_JUMP = 0x07, // 前轮附近第二个激光测距模块检测到车体高度突变
    ODOM_DATA = 0x08,                   // 麦克纳姆轮速反馈：<v_fl,v_rl,v_rr,v_fr>，单位: m/s
    ARM_HIGH_RAISE_DONE = 0x09,         // 梅林预选赛入口专属高抬升完成
    ARM_SECOND_LOWER_DONE = 0x0A,       // KFS 向下夹取前第二节机械臂放下完成
    ENTRY_GRAB_KFS_UP_DONE = 0x0B,      // 梅林预选赛入口高侧 KFS 夹取完成
    COMPETITION_START_DONE = 0x0C,      // 比赛开始状态切换完成
    SECOND_PRESELECTION_START_DONE = 0x0D, // 第二预选赛开始状态切换完成
    SECOND_PRESELECTION_ARM_HIGH_RAISE_DONE = 0x0F, // 第二预选赛机械臂高抬升完成
    MF_PRESELECTION_TRIGGER = 0x10,      // 第二限位开关事件；当前 gate profile 决定 0x10/0x0C 或 0x11/0x0D
    SECOND_PRESELECTION_PICKUP_KFS_DONE = 0x11, // 第二预选赛 KFS 夹取动作完成
    SECOND_PRESELECTION_ARM_LOWER_DONE = 0x12, // 第二预选赛机械臂彻底放下完成
    MCU_ERROR = 0xFE,                   // MCU 端错误码：下位机原因
};

// ============================================================================
// 0xFE 机械臂错误/状态反馈
// ============================================================================
enum class PlanarArmFailCode : uint8_t {
    BUSY = 0x01,            // 命令已接收，当前仍在执行中
    INVALID_PAYLOAD = 0x02, // payload 非法
    NOT_INIT = 0x03,        // 机械臂模块尚未初始化完成
    HAL_ERROR = 0x04,       // HAL 层或运动控制执行错误
    INVALID_STATE = 0x05,   // 当前状态不允许执行该命令
};

struct PlanarArmErrorFeedback {
    uint8_t failed_cmd{0};
    uint8_t error_code{0};
};

inline bool isPlanarArmErrorPayloadSize(size_t payload_size) {
    return payload_size == PLANAR_ARM_ERROR_PAYLOAD_SIZE;
}

inline const char* commandName(uint8_t command_id) {
    switch (static_cast<CommandID>(command_id)) {
    case CommandID::GRAB_TIP:
        return "GRAB_TIP";
    case CommandID::GRAB_KFS_DOWN:
        return "GRAB_KFS_DOWN";
    case CommandID::GRAB_KFS_UP:
        return "GRAB_KFS_UP";
    case CommandID::ARM_RAISE:
        return "ARM_RAISE";
    case CommandID::ARM_LOWER:
        return "ARM_LOWER";
    case CommandID::PLACE_KFS_GRID:
        return "PLACE_KFS_GRID";
    case CommandID::ARM_HIGH_RAISE:
        return "ARM_HIGH_RAISE";
    case CommandID::ARM_SECOND_LOWER:
        return "GRAB_KFS_DOWN_EXTEND";
    case CommandID::ENTRY_GRAB_KFS_UP:
        return "HIGH_CLAMP";
    case CommandID::COMPETITION_START:
        return "START";
    case CommandID::SECOND_PRESELECTION_START:
        return "BACK_CLAMP";
    case CommandID::SECOND_PRESELECTION_ARM_HIGH_RAISE:
        return "ARM_TOP_RAISE";
    case CommandID::SECOND_PRESELECTION_PLACE_KFS:
        return "CYLINDER_RELEASE";
    case CommandID::SECOND_PRESELECTION_ARM_LOWER:
        return "ARM_BOTTOM_LOWER";
    default:
        return "UNKNOWN_COMMAND";
    }
}

inline const char* planarArmFailCodeName(uint8_t error_code) {
    switch (static_cast<PlanarArmFailCode>(error_code)) {
    case PlanarArmFailCode::BUSY:
        return "PLANAR_ARM_FAIL_BUSY";
    case PlanarArmFailCode::INVALID_PAYLOAD:
        return "PLANAR_ARM_FAIL_INVALID_PAYLOAD";
    case PlanarArmFailCode::NOT_INIT:
        return "PLANAR_ARM_FAIL_NOT_INIT";
    case PlanarArmFailCode::HAL_ERROR:
        return "PLANAR_ARM_FAIL_HAL_ERROR";
    case PlanarArmFailCode::INVALID_STATE:
        return "PLANAR_ARM_FAIL_INVALID_STATE";
    default:
        return "PLANAR_ARM_FAIL_UNKNOWN";
    }
}

inline const char* planarArmFailCodeMeaning(uint8_t error_code) {
    switch (static_cast<PlanarArmFailCode>(error_code)) {
    case PlanarArmFailCode::BUSY:
        return "设备忙，命令仍在执行中";
    case PlanarArmFailCode::INVALID_PAYLOAD:
        return "命令 payload 非法";
    case PlanarArmFailCode::NOT_INIT:
        return "机械臂模块尚未初始化完成";
    case PlanarArmFailCode::HAL_ERROR:
        return "下位机 HAL 层或运动控制执行错误";
    case PlanarArmFailCode::INVALID_STATE:
        return "当前机械臂状态不允许执行该命令";
    default:
        return "未知机械臂错误码";
    }
}

inline const char* planarArmFailCodeRecommendation(uint8_t error_code) {
    switch (static_cast<PlanarArmFailCode>(error_code)) {
    case PlanarArmFailCode::BUSY:
        return "继续等待最终业务反馈，必要时低频重发同一 cmd+seq";
    case PlanarArmFailCode::INVALID_PAYLOAD:
        return "停止重发，检查上位机发送帧 payload 长度和格式";
    case PlanarArmFailCode::NOT_INIT:
        return "等待下位机初始化完成后再发送命令";
    case PlanarArmFailCode::HAL_ERROR:
        return "停止连续自动重发，进入异常保护或人工检查";
    case PlanarArmFailCode::INVALID_STATE:
        return "检查动作流程顺序，确认前置动作是否已完成";
    default:
        return "停止盲目重发，保留现场日志后排查";
    }
}

inline bool isPlanarArmBusy(uint8_t error_code) {
    return error_code == static_cast<uint8_t>(PlanarArmFailCode::BUSY);
}

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
using ::rc26_decision::PlanarArmErrorFeedback;
using ::rc26_decision::PlanarArmFailCode;
using ::rc26_decision::PLANAR_ARM_ERROR_PAYLOAD_SIZE;
using ::rc26_decision::commandName;
using ::rc26_decision::isPlanarArmBusy;
using ::rc26_decision::isPlanarArmErrorPayloadSize;
using ::rc26_decision::planarArmFailCodeMeaning;
using ::rc26_decision::planarArmFailCodeName;
using ::rc26_decision::planarArmFailCodeRecommendation;

}  // namespace rc26_serial
