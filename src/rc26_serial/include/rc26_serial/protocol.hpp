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
constexpr uint8_t MAX_RETRY_VALUE = 0x09;         // 最大重发次数值
constexpr uint8_t RETRIES_PER_ROUND = 3;          // 每轮重发次数
constexpr uint8_t MAX_PAYLOAD_SIZE = 32;          // payload 最大字节数
constexpr uint8_t MAX_RECONNECT_ATTEMPTS = 10;    // 最大重连尝试次数
constexpr uint8_t PLANAR_ARM_ERROR_PAYLOAD_SIZE = 2; // 0xFE 机械臂错误反馈 payload 字节数

// ============================================================================
// 下行指令 ID (上位机 -> MCU)
// ============================================================================
enum class CommandID : uint8_t {
    GRAB_TIP = 0x01,              // 抓取端头
    GRAB_KFS_DOWN = 0x02,         // 下台阶/下降方向 KFS 夹取
    GRAB_KFS_UP = 0x03,           // 上台阶/抬升方向 KFS 夹取
    ARM_RAISE = 0x04,             // 机械臂底座抬起
    ARM_LOWER = 0x05,             // 机械臂底座下降
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
    SECOND_PRESELECTION_PICKUP_KFS = 0x12, // 第二预选赛夹取平地 KFS
    SECOND_PRESELECTION_PLACE_KFS = 0x13,      // 第二预选赛放置 KFS（打开气缸）
    SECOND_PRESELECTION_ARM_LOWER = 0x14,      // 第二预选赛视觉对齐后机械臂彻底放下
    SECOND_PRESELECTION_PRELOAD_KFS_PICKUP = 0x15, // 第二预选赛夹出预装 KFS
    STARTUP_READY_WAITING_LIMIT = 0x20, // 深度相机已出帧且上位机正在等待人工限位触发
};

// ============================================================================
// 上行反馈 ID (MCU -> 上位机)
// ============================================================================
enum class FeedbackID : uint8_t {
    ACK = 0x00,                         // 确认收到指令
    ARM_RAISE_DONE = 0x02,              // 机械臂抬起完成
    ARM_LOWER_DONE = 0x03,              // 机械臂下降完成
    FRONT_LASER_HEIGHT_JUMP = 0x04,     // 前轮附近第一个激光测距模块检测到车体高度突变
    REAR_LASER_HEIGHT_JUMP = 0x05,      // 后轮附近激光测距模块检测到车体高度突变
    MANUAL_LIMIT_SWITCH_1_TRIGGERED = 0x06, // 人工触发外部限位 1，payload v1 为空或忽略
    FRONT_SECOND_LASER_HEIGHT_JUMP = 0x07, // 前轮附近第二个激光测距模块检测到车体高度突变
    ARM_HIGH_RAISE_DONE = 0x09,         // 梅林预选赛入口专属高抬升完成
    ARM_SECOND_LOWER_DONE = 0x0A,       // KFS 向下夹取前第二节机械臂放下完成
    ENTRY_GRAB_KFS_UP_DONE = 0x0B,      // 梅林预选赛入口高侧 KFS 夹取完成
    COMPETITION_START_DONE = 0x0C,      // 比赛开始状态切换完成
    SECOND_PRESELECTION_START_DONE = 0x0D, // 第二预选赛开始状态切换完成
    MANUAL_LIMIT_SWITCH_2_TRIGGERED = 0x10, // 人工触发外部限位 2
    SECOND_PRESELECTION_PICKUP_KFS_DONE = 0x11, // 第二预选赛 KFS 夹取动作完成
    SECOND_PRESELECTION_ARM_LOWER_DONE = 0x12, // 第二预选赛机械臂彻底放下完成
    MANUAL_LIMIT_SWITCH_3_TRIGGERED = 0x13, // 人工触发外部限位 3，切换下一次启动的 active_side
    SECOND_PRESELECTION_PRELOAD_KFS_PICKUP_DONE = 0x14, // 第二预选赛预装 KFS 夹取完成
    SECOND_PRESELECTION_MANUAL_FRONT_LASER_TRIGGERED = 0x15, // 第二预选赛人工触发前轮激光成功
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
    case CommandID::FRONT_PUSHROD_EXTEND:
        return "FRONT_PUSHROD_EXTEND";
    case CommandID::FRONT_PUSHROD_RETRACT:
        return "FRONT_PUSHROD_RETRACT";
    case CommandID::REAR_PUSHROD_EXTEND:
        return "REAR_PUSHROD_EXTEND";
    case CommandID::REAR_PUSHROD_RETRACT:
        return "REAR_PUSHROD_RETRACT";
    case CommandID::POSE_TARGET:
        return "POSE_TARGET";
    case CommandID::ARM_HIGH_RAISE:
        return "ARM_HIGH_RAISE";
    case CommandID::ARM_SECOND_LOWER:
        return "ARM_SECOND_LOWER";
    case CommandID::ENTRY_GRAB_KFS_UP:
        return "ENTRY_GRAB_KFS_UP";
    case CommandID::COMPETITION_START:
        return "COMPETITION_START";
    case CommandID::SECOND_PRESELECTION_START:
        return "SECOND_PRESELECTION_START";
    case CommandID::SECOND_PRESELECTION_PICKUP_KFS:
        return "SECOND_PRESELECTION_PICKUP_KFS";
    case CommandID::SECOND_PRESELECTION_PLACE_KFS:
        return "SECOND_PRESELECTION_PLACE_KFS";
    case CommandID::SECOND_PRESELECTION_ARM_LOWER:
        return "SECOND_PRESELECTION_ARM_LOWER";
    case CommandID::SECOND_PRESELECTION_PRELOAD_KFS_PICKUP:
        return "SECOND_PRESELECTION_PRELOAD_KFS_PICKUP";
    case CommandID::STARTUP_READY_WAITING_LIMIT:
        return "STARTUP_READY_WAITING_LIMIT";
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
using ::rc26_decision::PlanarArmFailCode;
using ::rc26_decision::PLANAR_ARM_ERROR_PAYLOAD_SIZE;
using ::rc26_decision::commandName;
using ::rc26_decision::isPlanarArmBusy;
using ::rc26_decision::isPlanarArmErrorPayloadSize;
using ::rc26_decision::planarArmFailCodeMeaning;
using ::rc26_decision::planarArmFailCodeName;
using ::rc26_decision::planarArmFailCodeRecommendation;

}  // namespace rc26_serial
