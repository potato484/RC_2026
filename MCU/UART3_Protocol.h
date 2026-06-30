#ifndef UART3_PROTOCOL_H
#define UART3_PROTOCOL_H

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 帧头固定标识 */
#define UART3_PROTOCOL_HEAD_BYTE_0      0xAAU
#define UART3_PROTOCOL_HEAD_BYTE_1      0x55U
/* 帧尾固定标识 */
#define UART3_PROTOCOL_TAIL_BYTE_0      0x55U
#define UART3_PROTOCOL_TAIL_BYTE_1      0xAAU
/* 协议允许的最大有效载荷长度 */
#define UART3_PROTOCOL_MAX_PAYLOAD_SIZE 32U
/* 协议单帧最大总长度，含头、控制字段、CRC 与帧尾 */
#define UART3_PROTOCOL_MAX_FRAME_SIZE   44U

/* 协议内建确认帧命令 ID */
#define UART3_PROTOCOL_ACK_ID           0x00U
/* 协议内建否认帧命令 ID */
#define UART3_PROTOCOL_NACK_ID          0x0AU

typedef struct
{
    /* 帧序号，用于主从收发匹配 */
    uint8_t seq;
    /* 协议长度字段，值为 cmd + payload 总长度 */
    uint8_t len;
    /* 重发计数或重试标记 */
    uint8_t retry;
    /* 命令字 */
    uint8_t cmd;
    /* 解包后的有效载荷长度 */
    uint8_t payload_len;
    /* 有效载荷数据 */
    uint8_t payload[UART3_PROTOCOL_MAX_PAYLOAD_SIZE];
} UART3_Protocol_Frame_t;

typedef enum
{
    /* 当前命令处理成功，协议层自动回复 ACK */
    UART3_PROTOCOL_HANDLER_ACK = 0U,
    /* 当前命令处理失败，协议层自动回复 NACK */
    UART3_PROTOCOL_HANDLER_NACK = 1U,
    /* 当前命令延后处理，由上层自行决定是否回复 */
    UART3_PROTOCOL_HANDLER_DEFER = 2U,
} UART3_Protocol_HandlerResult_t;

/* 协议命令处理回调，context 为注册时绑定的用户上下文 */
typedef UART3_Protocol_HandlerResult_t (*UART3_Protocol_CommandHandler_t)(const UART3_Protocol_Frame_t *frame,
                                                                          void *context);

/* 初始化 UART3 协议模块，启动 DMA 空闲接收与内部状态机 */
void UART3_Protocol_Init(void);
/* 轮询处理接收环形缓冲与发送队列 */
void UART3_Protocol_Poll(void);
/* 发送业务反馈帧，feedback_id 作为命令字透传 */
HAL_StatusTypeDef UART3_Protocol_SendFeedback(uint8_t seq,
                                              uint8_t feedback_id,
                                              const uint8_t *payload,
                                              uint8_t payload_len);
/* 发送 ACK 控制帧 */
HAL_StatusTypeDef UART3_Protocol_SendAck(uint8_t seq);
/* 发送 NACK 控制帧 */
HAL_StatusTypeDef UART3_Protocol_SendNack(uint8_t seq);
/* 注册指定命令字处理函数 */
HAL_StatusTypeDef UART3_Protocol_RegisterHandler(uint8_t cmd, UART3_Protocol_CommandHandler_t handler, void *context);

#ifdef __cplusplus
}
#endif

#endif
