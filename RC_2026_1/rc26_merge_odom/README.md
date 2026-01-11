# rc26_merge_odom

RC2026 里程计融合与IMU驱动包，提供CAN里程计、位姿下传、达妙IMU驱动功能。

## 功能模块

### 1. CAN里程计 (can_odom)

从SocketCAN接口读取轮式里程计数据，发布`nav_msgs/Odometry`。

### 2. 位姿下传 (pose_sender)

将上位机位姿数据通过串口发送给下位机MCU。

### 3. 达妙IMU驱动 (dm_imu_driver)

达妙科技 DM-IMU-L1 六轴IMU驱动，支持USB串口通信。

---

## 达妙IMU

### 产品概述

DM-IMU-L1 是达妙科技的六轴IMU模块：
- 传感器：BMI088 (三轴加速度计 + 三轴陀螺仪)
- 内置EKF四元数姿态解算
- 支持USB/RS485/CAN输出
- 输出频率：100-1000Hz可调
- 自带加热电路实现恒温控制

### 协议格式 (USB通信)

基于 **DM-IMU-L1 使用说明书 V1.2**

```
┌──────┬──────┬────┬─────┬────────────┬───────┬──────┐
│ HEAD │ HEAD │ ID │ RID │   DATA     │ CRC16 │ TAIL │
│ 0x55 │ 0xAA │ 1B │ 1B  │ 12B(3xf32) │  2B   │ 0x0A │
└──────┴──────┴────┴─────┴────────────┴───────┴──────┘
```

| 字段 | 长度 | 说明 |
|------|------|------|
| HEAD | 2B | 帧头 `0x55 0xAA` |
| ID | 1B | 从机ID（与CAN/485的ID一致） |
| RID | 1B | 数据类型标识 |
| DATA | 12B | 3个float32，小端序 |
| CRC16 | 2B | CRC16校验，小端序 |
| TAIL | 1B | 帧尾 `0x0A` |

**帧长度**: 固定19字节

### RID数据类型

| RID | 类型 | DATA[0] | DATA[1] | DATA[2] | 单位 |
|-----|------|---------|---------|---------|------|
| `0x01` | 加速度 | Acc_X | Acc_Y | Acc_Z | m/s² |
| `0x02` | 角速度 | Gyro_X | Gyro_Y | Gyro_Z | °/s |
| `0x03` | 欧拉角 | Roll | Pitch | Yaw | ° |
| `0x04` | 四元数 | W,X,Y,Z (特殊编码) | - | - | - |

### 参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `port` | `/dev/ttyACM0` | 串口设备路径 |
| `baudrate` | `921600` | 波特率 |
| `frame_id` | `imu_link` | IMU坐标系名称 |
| `verbose` | `false` | 详细日志输出 |
| `qos_reliable` | `true` | QoS可靠传输 |

### 话题

| 话题 | 类型 | 说明 |
|------|------|------|
| `/DM_IMU` | `sensor_msgs/Imu` | IMU数据输出 |

### 使用方式

```bash
# 启动文件
ros2 launch rc26_merge_odom dm_imu.launch.py

# 自定义参数
ros2 launch rc26_merge_odom dm_imu.launch.py port:=/dev/ttyACM1 verbose:=true

# 直接运行节点
ros2 run rc26_merge_odom dm_imu_node --ros-args -p port:=/dev/ttyACM0
```

### API 使用

```cpp
#include "rc26_merge_odom/imu/dm_imu_driver.hpp"

rc26_merge_odom::DmImuDriver driver;

// 打开串口
if (!driver.open("/dev/ttyACM0", 921600)) {
    // 处理错误
}

// 设置数据回调
driver.setDataCallback([](const rc26_merge_odom::ImuData& data) {
    if (data.euler_valid) {
        printf("Roll=%.2f Pitch=%.2f Yaw=%.2f\n",
               data.euler[0], data.euler[1], data.euler[2]);
    }
});

// 或轮询获取数据
rc26_merge_odom::ImuData data = driver.getLatestData();

// 获取统计信息
auto stats = driver.getStats();
printf("OK=%u CRC_ERR=%u\n", stats.frames_ok, stats.frames_crc_error);

// 关闭
driver.close();
```

## 节点

| 节点 | 说明 |
|------|------|
| `can_odom_node` | CAN里程计节点 |
| `pose_sender_node` | 位姿下传节点 |
| `merge_odom_node` | 融合里程计主节点 |
| `dm_imu_node` | 达妙IMU节点 |

## 启动文件

| 文件 | 说明 |
|------|------|
| `dm_imu.launch.py` | 启动达妙IMU节点 |
| `can_odom_only.launch.py` | 仅启动CAN里程计 |
| `merge_odom.launch.py` | 启动融合里程计 |

## 文件结构

```
rc26_merge_odom/
├── include/rc26_merge_odom/
│   ├── can_odom.hpp          # CAN里程计头文件
│   ├── pose_sender.hpp       # 位姿下传头文件
│   ├── dm_protocol.hpp       # 达妙IMU协议定义
│   └── dm_imu_driver.hpp     # 达妙IMU驱动头文件
├── src/
│   ├── can_odom.cpp          # CAN里程计实现
│   ├── can_odom_node.cpp     # CAN里程计节点
│   ├── pose_sender.cpp       # 位姿下传实现
│   ├── pose_sender_node.cpp  # 位姿下传节点
│   ├── merge_odom_node.cpp   # 融合里程计节点
│   ├── dm_imu_driver.cpp     # 达妙IMU驱动实现
│   └── dm_imu_node.cpp       # 达妙IMU节点
├── launch/
│   ├── dm_imu.launch.py
│   ├── can_odom_only.launch.py
│   └── merge_odom.launch.py
├── config/
│   └── ...
├── CMakeLists.txt
├── package.xml
└── README.md
```

## 依赖

- rclcpp
- std_msgs
- nav_msgs
- sensor_msgs
- geometry_msgs
- tf2
- tf2_ros
- rc26_serial

## 编译

```bash
colcon build --packages-select rc26_merge_odom
```

---

## 底层原理

### 1. 串口配置

与`rc26_serial`相同，使用Linux原生`termios` API：
- **8N1模式**: 8数据位、无校验、1停止位
- **原始模式**: 禁用行缓冲、回显
- **无流控**: 禁用硬件/软件流控

### 2. 异步读取

独立线程 + `select()` 非阻塞读取：

```
┌─────────────┐     ┌──────────────┐     ┌─────────────┐
│ recv_thread │────▶│ select(10ms) │────▶│ read(fd)    │
└─────────────┘     └──────────────┘     └─────────────┘
                                                │
                                                ▼
                                    ┌──────────────────┐
                                    │ parseReceivedData│
                                    └──────────────────┘
```

### 3. 帧解析流程

```
┌─────────────────────────────────────────────────────────────┐
│ recv_buffer_                                                 │
│ [...] [0x55] [0xAA] [ID] [RID] [DATA...] [CRC16] [0x0A] [...] │
└─────────────────────────────────────────────────────────────┘
         │
         ▼
   ┌─────────────┐
   │ 查找帧头     │ find(0x55 0xAA)
   │ 0x55 0xAA   │
   └─────────────┘
         │
         ▼
   ┌─────────────┐
   │ 检查帧长度   │ recv_len >= 19?
   │ 19字节固定   │
   └─────────────┘
         │
         ▼
   ┌─────────────┐
   │ 检查帧尾     │ frame[18] == 0x0A?
   │ 0x0A        │
   └─────────────┘
         │
         ▼
   ┌─────────────┐
   │ CRC16校验   │ dm_crc16(frame[0:16]) == frame[16:18]?
   └─────────────┘
         │
         ▼
   ┌─────────────┐
   │ 解析数据     │ 按RID分类存储
   └─────────────┘
```

### 4. CRC16 校验

```cpp
// CRC16 查表算法（来自达妙说明书附录四）
uint16_t dm_crc16(const uint8_t* data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; ++i) {
        uint8_t index = ((crc >> 8) ^ data[i]) & 0xFF;
        crc = ((crc << 1) ^ CRC16_TABLE[index]) & 0xFFFF;
    }
    return crc;
}
```

**校验范围**: `HEAD(2) + ID(1) + RID(1) + DATA(12)` = 前16字节

### 5. 数据结构

```cpp
struct ImuData {
    float accel[3];      // 加速度 X/Y/Z (m/s²)
    float gyro[3];       // 角速度 X/Y/Z (°/s)
    float euler[3];      // 欧拉角 Roll/Pitch/Yaw (°)
    float quaternion[4]; // 四元数 W/X/Y/Z
    
    bool accel_valid;    // 各数据有效标志
    bool gyro_valid;
    bool euler_valid;
    bool quaternion_valid;
    
    double accel_ts;     // 各数据时间戳
    double gyro_ts;
    double euler_ts;
    double quaternion_ts;
};
```

---

## 达妙IMU快捷指令参考

需通过USB串口发送（十六进制），**除重启外均需先进入设置模式**：

### 模式切换

| 指令 | 说明 |
|------|------|
| `AA 06 01 0D` | 进入设置模式（黄灯呼吸） |
| `AA 06 00 0D` | 进入正常模式（绿灯呼吸） |
| `AA 00 00 0D` | 重启IMU |

### 校准指令

| 指令 | 说明 |
|------|------|
| `AA 0C 01 0D` | 角度置零 |
| `AA 03 02 0D` | 启动陀螺静态校准 |
| `AA 03 03 0D` | 启动加速度计六面校准 |

### 输出控制

| 指令 | 说明 |
|------|------|
| `AA 01 14 0D` | 开启加速度输出 |
| `AA 01 04 0D` | 关闭加速度输出 |
| `AA 01 15 0D` | 开启角速度输出 |
| `AA 01 05 0D` | 关闭角速度输出 |
| `AA 01 16 0D` | 开启欧拉角输出 |
| `AA 01 06 0D` | 关闭欧拉角输出 |
| `AA 01 17 0D` | 开启四元数输出 |
| `AA 01 07 0D` | 关闭四元数输出 |

### 参数设置

| 指令 | 说明 |
|------|------|
| `AA 03 01 0D` | 保存参数 |
| `AA 0B 01 0D` | 恢复出厂设置 |
| `AA 04 01 0D` | 开启温度控制 |
| `AA 04 00 0D` | 关闭温度控制 |
| `AA 02 XX XX 0D` | 设置反馈间隔（2字节，小端序） |

### 通信设置

| 指令 | 说明 |
|------|------|
| `AA 0A 00 0D` | 输出接口：USB |
| `AA 0A 01 0D` | 输出接口：485 |
| `AA 0A 02 0D` | 输出接口：CAN |
| `AA 08 XX 0D` | 设置CAN ID |
| `AA 09 XX 0D` | 设置MST ID |

### 指示灯状态

| 状态 | 说明 |
|------|------|
| 绿灯呼吸 | 正常模式 |
| 黄灯呼吸 | 设置模式 |
| 红灯常亮 | 错误状态 |
| 黄灯闪烁N次 | 加速度计第N面校准中 |

---

## 作者

RC2026 Team
