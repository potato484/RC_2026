#include "UART3_Protocol.h"

#include "usart.h"

#include <string.h>

/*
 * UART3 协议模块
 * 数据流：USART3 DMA 空闲接收 -> 接收环形缓冲 -> 流式拼帧 -> CRC32/帧尾校验
 *       -> 命令分发 -> 自动 ACK/NACK -> DMA 发送队列输出
 */

#define UART3_PROTOCOL_RX_DMA_BUFFER_SIZE    128U
#define UART3_PROTOCOL_RX_RING_BUFFER_SIZE   256U
#define UART3_PROTOCOL_TX_QUEUE_DEPTH        8U
#define UART3_PROTOCOL_CRC_TABLE_SIZE        256U
#define UART3_PROTOCOL_MIN_LEN_FIELD_VALUE   1U
#define UART3_PROTOCOL_MAX_LEN_FIELD_VALUE   (UART3_PROTOCOL_MAX_PAYLOAD_SIZE + 1U)
#define UART3_PROTOCOL_FIXED_OVERHEAD_SIZE   12U
#define UART3_PROTOCOL_CRC_SEED              0xFFFFFFFFUL
#define UART3_PROTOCOL_CRC_POLYNOMIAL        0x04C11DB7UL

typedef struct
{
    uint8_t data[UART3_PROTOCOL_MAX_FRAME_SIZE];
    uint8_t length;
} UART3_Protocol_TxNode_t;

typedef struct
{
    UART3_Protocol_CommandHandler_t handler;
    void *context;
} UART3_Protocol_HandlerSlot_t;

typedef struct
{
    uint32_t invalid_head_count;
    uint32_t invalid_length_count;
    uint32_t crc_error_count;
    uint32_t tail_error_count;
    uint32_t rx_ring_overflow_count;
    uint32_t parse_overflow_count;
    uint32_t rx_restart_error_count;
    uint32_t tx_start_error_count;
} UART3_Protocol_InternalStats_t;

static uint32_t s_crc32_table[UART3_PROTOCOL_CRC_TABLE_SIZE];
static uint8_t s_crc32_table_ready = 0U;

static uint8_t s_rx_dma_buffer[UART3_PROTOCOL_RX_DMA_BUFFER_SIZE];
static volatile uint16_t s_rx_dma_last_pos = 0U;

static uint8_t s_rx_ring_buffer[UART3_PROTOCOL_RX_RING_BUFFER_SIZE];
static volatile uint16_t s_rx_ring_head = 0U;
static volatile uint16_t s_rx_ring_tail = 0U;

static uint8_t s_parse_buffer[UART3_PROTOCOL_MAX_FRAME_SIZE];
static uint8_t s_parse_length = 0U;

static UART3_Protocol_TxNode_t s_tx_queue[UART3_PROTOCOL_TX_QUEUE_DEPTH];
static volatile uint8_t s_tx_queue_head = 0U;
static volatile uint8_t s_tx_queue_tail = 0U;
static volatile uint8_t s_tx_queue_count = 0U;
static volatile uint8_t s_tx_dma_busy = 0U;

static UART3_Protocol_HandlerSlot_t s_handler_slots[256];

static UART3_Protocol_InternalStats_t s_protocol_stats = {0};

static void UART3_Protocol_InitCrc32Table(void);
static uint32_t UART3_Protocol_CalcCrc32(const uint8_t *data, uint16_t length);
static HAL_StatusTypeDef UART3_Protocol_StartRxDma(void);
static void UART3_Protocol_HandleRxDmaEvent(uint16_t dma_pos);
static void UART3_Protocol_RxRingPushByte(uint8_t byte);
static uint8_t UART3_Protocol_RxRingPopByte(uint8_t *byte);
static void UART3_Protocol_StreamFeedByte(uint8_t byte);
static void UART3_Protocol_TryParseFrames(void);
static void UART3_Protocol_RemoveParsePrefix(uint8_t count);
static void UART3_Protocol_HandleDecodedFrame(const uint8_t *frame_data);
static HAL_StatusTypeDef UART3_Protocol_SendControlFrame(uint8_t seq, uint8_t cmd);
static HAL_StatusTypeDef UART3_Protocol_BuildFrame(uint8_t seq,
                                                   uint8_t retry,
                                                   uint8_t cmd,
                                                   const uint8_t *payload,
                                                   uint8_t payload_len,
                                                   uint8_t *frame_buffer,
                                                   uint8_t *frame_length);
static HAL_StatusTypeDef UART3_Protocol_EnqueueTxFrame(const uint8_t *frame_data, uint8_t frame_length);
static HAL_StatusTypeDef UART3_Protocol_StartNextTx(void);

void UART3_Protocol_Init(void)
{
    UART3_Protocol_InitCrc32Table();

    s_rx_dma_last_pos = 0U;
    s_rx_ring_head = 0U;
    s_rx_ring_tail = 0U;
    s_parse_length = 0U;
    s_tx_queue_head = 0U;
    s_tx_queue_tail = 0U;
    s_tx_queue_count = 0U;
    s_tx_dma_busy = 0U;

    memset(s_rx_dma_buffer, 0, sizeof(s_rx_dma_buffer));
    memset(s_rx_ring_buffer, 0, sizeof(s_rx_ring_buffer));
    memset(s_parse_buffer, 0, sizeof(s_parse_buffer));
    memset(s_tx_queue, 0, sizeof(s_tx_queue));
    memset(s_handler_slots, 0, sizeof(s_handler_slots));
    memset(&s_protocol_stats, 0, sizeof(s_protocol_stats));

    (void)UART3_Protocol_StartRxDma();
}

void UART3_Protocol_Poll(void)
{
    uint8_t byte = 0U;

    while (UART3_Protocol_RxRingPopByte(&byte) != 0U)
    {
        UART3_Protocol_StreamFeedByte(byte);
    }

    (void)UART3_Protocol_StartNextTx();
}

HAL_StatusTypeDef UART3_Protocol_SendFeedback(uint8_t seq,
                                              uint8_t feedback_id,
                                              const uint8_t *payload,
                                              uint8_t payload_len)
{
    uint8_t frame_buffer[UART3_PROTOCOL_MAX_FRAME_SIZE];
    uint8_t frame_length = 0U;

    if ((payload_len > 0U) && (payload == NULL))
    {
        return HAL_ERROR;
    }

    if (UART3_Protocol_BuildFrame(seq, 0U, feedback_id, payload, payload_len, frame_buffer, &frame_length) != HAL_OK)
    {
        return HAL_ERROR;
    }

    return UART3_Protocol_EnqueueTxFrame(frame_buffer, frame_length);
}

HAL_StatusTypeDef UART3_Protocol_SendAck(uint8_t seq)
{
    return UART3_Protocol_SendControlFrame(seq, UART3_PROTOCOL_ACK_ID);
}

HAL_StatusTypeDef UART3_Protocol_SendNack(uint8_t seq)
{
    return UART3_Protocol_SendControlFrame(seq, UART3_PROTOCOL_NACK_ID);
}

HAL_StatusTypeDef UART3_Protocol_RegisterHandler(uint8_t cmd, UART3_Protocol_CommandHandler_t handler, void *context)
{
    if (handler == NULL)
    {
        return HAL_ERROR;
    }

    s_handler_slots[cmd].handler = handler;
    s_handler_slots[cmd].context = context;

    return HAL_OK;
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if ((huart != NULL) && (huart->Instance == USART3))
    {
        /* DMA 空闲/满缓冲事件：转存新增字节并立即重启接收 */
        UART3_Protocol_HandleRxDmaEvent(Size);
        if (UART3_Protocol_StartRxDma() != HAL_OK)
        {
            s_protocol_stats.rx_restart_error_count++;
        }
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if ((huart != NULL) && (huart->Instance == USART3))
    {
        /* 单帧 DMA 发送完成：出队当前节点并触发后续待发帧 */
        __disable_irq();
        if (s_tx_queue_count > 0U)
        {
            s_tx_queue_head = (uint8_t)((s_tx_queue_head + 1U) % UART3_PROTOCOL_TX_QUEUE_DEPTH);
            s_tx_queue_count--;
        }
        s_tx_dma_busy = 0U;
        __enable_irq();

        (void)UART3_Protocol_StartNextTx();
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if ((huart != NULL) && (huart->Instance == USART3))
    {
        /* USART3 异常后停止 DMA，并尝试恢复接收链路 */
        (void)HAL_UART_DMAStop(huart);
        if (UART3_Protocol_StartRxDma() != HAL_OK)
        {
            s_protocol_stats.rx_restart_error_count++;
        }
    }
}

static void UART3_Protocol_InitCrc32Table(void)
{
    uint32_t i = 0U;

    if (s_crc32_table_ready != 0U)
    {
        return;
    }

    for (i = 0U; i < UART3_PROTOCOL_CRC_TABLE_SIZE; i++)
    {
        uint32_t crc = i << 24U;
        uint8_t bit = 0U;

        for (bit = 0U; bit < 8U; bit++)
        {
            if ((crc & 0x80000000UL) != 0U)
            {
                crc = (crc << 1U) ^ UART3_PROTOCOL_CRC_POLYNOMIAL;
            }
            else
            {
                crc <<= 1U;
            }
        }

        s_crc32_table[i] = crc;
    }

    s_crc32_table_ready = 1U;
}

static uint32_t UART3_Protocol_CalcCrc32(const uint8_t *data, uint16_t length)
{
    uint32_t crc = UART3_PROTOCOL_CRC_SEED;

    while (length > 0U)
    {
        uint8_t index = (uint8_t)((crc >> 24U) ^ *data);
        crc = (crc << 8U) ^ s_crc32_table[index];
        data++;
        length--;
    }

    return crc;
}

static HAL_StatusTypeDef UART3_Protocol_StartRxDma(void)
{
    s_rx_dma_last_pos = 0U;

    if (HAL_UARTEx_ReceiveToIdle_DMA(&huart3, s_rx_dma_buffer, UART3_PROTOCOL_RX_DMA_BUFFER_SIZE) != HAL_OK)
    {
        return HAL_ERROR;
    }

    if (huart3.hdmarx != NULL)
    {
        __HAL_DMA_DISABLE_IT(huart3.hdmarx, DMA_IT_HT);
    }

    return HAL_OK;
}

static void UART3_Protocol_HandleRxDmaEvent(uint16_t dma_pos)
{
    uint16_t copy_index = 0U;

    if (dma_pos > UART3_PROTOCOL_RX_DMA_BUFFER_SIZE)
    {
        dma_pos = UART3_PROTOCOL_RX_DMA_BUFFER_SIZE;
    }

    if (dma_pos == s_rx_dma_last_pos)
    {
        return;
    }

    if (dma_pos > s_rx_dma_last_pos)
    {
        for (copy_index = s_rx_dma_last_pos; copy_index < dma_pos; copy_index++)
        {
            UART3_Protocol_RxRingPushByte(s_rx_dma_buffer[copy_index]);
        }
    }
    else
    {
        for (copy_index = s_rx_dma_last_pos; copy_index < UART3_PROTOCOL_RX_DMA_BUFFER_SIZE; copy_index++)
        {
            UART3_Protocol_RxRingPushByte(s_rx_dma_buffer[copy_index]);
        }

        for (copy_index = 0U; copy_index < dma_pos; copy_index++)
        {
            UART3_Protocol_RxRingPushByte(s_rx_dma_buffer[copy_index]);
        }
    }

    s_rx_dma_last_pos = dma_pos;
}

static void UART3_Protocol_RxRingPushByte(uint8_t byte)
{
    uint16_t next_head = (uint16_t)((s_rx_ring_head + 1U) % UART3_PROTOCOL_RX_RING_BUFFER_SIZE);

    if (next_head == s_rx_ring_tail)
    {
        s_protocol_stats.rx_ring_overflow_count++;
        return;
    }

    s_rx_ring_buffer[s_rx_ring_head] = byte;
    s_rx_ring_head = next_head;
}

static uint8_t UART3_Protocol_RxRingPopByte(uint8_t *byte)
{
    if ((byte == NULL) || (s_rx_ring_head == s_rx_ring_tail))
    {
        return 0U;
    }

    *byte = s_rx_ring_buffer[s_rx_ring_tail];
    s_rx_ring_tail = (uint16_t)((s_rx_ring_tail + 1U) % UART3_PROTOCOL_RX_RING_BUFFER_SIZE);

    return 1U;
}

static void UART3_Protocol_StreamFeedByte(uint8_t byte)
{
    if (s_parse_length >= UART3_PROTOCOL_MAX_FRAME_SIZE)
    {
        s_protocol_stats.parse_overflow_count++;
        UART3_Protocol_RemoveParsePrefix(1U);
    }

    s_parse_buffer[s_parse_length] = byte;
    s_parse_length++;

    UART3_Protocol_TryParseFrames();
}

static void UART3_Protocol_TryParseFrames(void)
{
    while (s_parse_length >= 2U)
    {
        uint8_t len_field = 0U;
        uint8_t payload_len = 0U;
        uint8_t frame_length = 0U;
        uint8_t crc_offset = 0U;
        uint32_t expected_crc = 0U;
        uint32_t actual_crc = 0U;

        if ((s_parse_buffer[0] != UART3_PROTOCOL_HEAD_BYTE_0) ||
            (s_parse_buffer[1] != UART3_PROTOCOL_HEAD_BYTE_1))
        {
            s_protocol_stats.invalid_head_count++;
            UART3_Protocol_RemoveParsePrefix(1U);
            continue;
        }

        if (s_parse_length < 4U)
        {
            break;
        }

        len_field = s_parse_buffer[3];
        if ((len_field < UART3_PROTOCOL_MIN_LEN_FIELD_VALUE) ||
            (len_field > UART3_PROTOCOL_MAX_LEN_FIELD_VALUE))
        {
            s_protocol_stats.invalid_length_count++;
            UART3_Protocol_RemoveParsePrefix(1U);
            continue;
        }

        payload_len = (uint8_t)(len_field - 1U);
        frame_length = (uint8_t)(len_field + 11U);

        if (s_parse_length < frame_length)
        {
            break;
        }

        crc_offset = (uint8_t)(6U + payload_len);
        expected_crc = (uint32_t)s_parse_buffer[crc_offset] |
                       ((uint32_t)s_parse_buffer[crc_offset + 1U] << 8U) |
                       ((uint32_t)s_parse_buffer[crc_offset + 2U] << 16U) |
                       ((uint32_t)s_parse_buffer[crc_offset + 3U] << 24U);

        actual_crc = UART3_Protocol_CalcCrc32(&s_parse_buffer[2], (uint16_t)(len_field + 3U));
        if (actual_crc != expected_crc)
        {
            s_protocol_stats.crc_error_count++;
            UART3_Protocol_RemoveParsePrefix(1U);
            continue;
        }

        if ((s_parse_buffer[frame_length - 2U] != UART3_PROTOCOL_TAIL_BYTE_0) ||
            (s_parse_buffer[frame_length - 1U] != UART3_PROTOCOL_TAIL_BYTE_1))
        {
            s_protocol_stats.tail_error_count++;
            UART3_Protocol_RemoveParsePrefix(1U);
            continue;
        }

        UART3_Protocol_HandleDecodedFrame(s_parse_buffer);
        UART3_Protocol_RemoveParsePrefix(frame_length);
    }
}

static void UART3_Protocol_RemoveParsePrefix(uint8_t count)
{
    if (count >= s_parse_length)
    {
        s_parse_length = 0U;
        return;
    }

    memmove(s_parse_buffer, &s_parse_buffer[count], s_parse_length - count);
    s_parse_length = (uint8_t)(s_parse_length - count);
}

static void UART3_Protocol_HandleDecodedFrame(const uint8_t *frame_data)
{
    UART3_Protocol_Frame_t frame = {0};
    UART3_Protocol_HandlerResult_t handler_result = UART3_PROTOCOL_HANDLER_NACK;
    UART3_Protocol_CommandHandler_t handler = NULL;

    frame.seq = frame_data[2];
    frame.len = frame_data[3];
    frame.retry = frame_data[4];
    frame.cmd = frame_data[5];
    frame.payload_len = (uint8_t)(frame.len - 1U);

    if (frame.payload_len > 0U)
    {
        memcpy(frame.payload, &frame_data[6], frame.payload_len);
    }

    handler = s_handler_slots[frame.cmd].handler;
    if (handler != NULL)
    {
        handler_result = handler(&frame, s_handler_slots[frame.cmd].context);
    }

    if (handler_result == UART3_PROTOCOL_HANDLER_ACK)
    {
        (void)UART3_Protocol_SendAck(frame.seq);
    }
    else if (handler_result == UART3_PROTOCOL_HANDLER_NACK)
    {
        (void)UART3_Protocol_SendNack(frame.seq);
    }
    else
    {
        /* DEFER: handled by upper-layer logic */
    }
}

static HAL_StatusTypeDef UART3_Protocol_SendControlFrame(uint8_t seq, uint8_t cmd)
{
    uint8_t frame_buffer[UART3_PROTOCOL_MAX_FRAME_SIZE];
    uint8_t frame_length = 0U;

    if (UART3_Protocol_BuildFrame(seq, 0U, cmd, NULL, 0U, frame_buffer, &frame_length) != HAL_OK)
    {
        return HAL_ERROR;
    }

    return UART3_Protocol_EnqueueTxFrame(frame_buffer, frame_length);
}

static HAL_StatusTypeDef UART3_Protocol_BuildFrame(uint8_t seq,
                                                   uint8_t retry,
                                                   uint8_t cmd,
                                                   const uint8_t *payload,
                                                   uint8_t payload_len,
                                                   uint8_t *frame_buffer,
                                                   uint8_t *frame_length)
{
    uint8_t len_field = 0U;
    uint8_t crc_offset = 0U;
    uint32_t crc_value = 0U;

    if ((frame_buffer == NULL) || (frame_length == NULL) || (payload_len > UART3_PROTOCOL_MAX_PAYLOAD_SIZE))
    {
        return HAL_ERROR;
    }

    if ((payload_len > 0U) && (payload == NULL))
    {
        return HAL_ERROR;
    }

    len_field = (uint8_t)(payload_len + 1U);

    frame_buffer[0] = UART3_PROTOCOL_HEAD_BYTE_0;
    frame_buffer[1] = UART3_PROTOCOL_HEAD_BYTE_1;
    frame_buffer[2] = seq;
    frame_buffer[3] = len_field;
    frame_buffer[4] = retry;
    frame_buffer[5] = cmd;

    if (payload_len > 0U)
    {
        memcpy(&frame_buffer[6], payload, payload_len);
    }

    crc_value = UART3_Protocol_CalcCrc32(&frame_buffer[2], (uint16_t)(len_field + 3U));
    crc_offset = (uint8_t)(6U + payload_len);
    frame_buffer[crc_offset] = (uint8_t)(crc_value & 0xFFU);
    frame_buffer[crc_offset + 1U] = (uint8_t)((crc_value >> 8U) & 0xFFU);
    frame_buffer[crc_offset + 2U] = (uint8_t)((crc_value >> 16U) & 0xFFU);
    frame_buffer[crc_offset + 3U] = (uint8_t)((crc_value >> 24U) & 0xFFU);
    frame_buffer[crc_offset + 4U] = UART3_PROTOCOL_TAIL_BYTE_0;
    frame_buffer[crc_offset + 5U] = UART3_PROTOCOL_TAIL_BYTE_1;

    *frame_length = (uint8_t)(payload_len + UART3_PROTOCOL_FIXED_OVERHEAD_SIZE);

    return HAL_OK;
}

static HAL_StatusTypeDef UART3_Protocol_EnqueueTxFrame(const uint8_t *frame_data, uint8_t frame_length)
{
    uint8_t enqueue_index = 0U;

    if ((frame_data == NULL) ||
        (frame_length < UART3_PROTOCOL_FIXED_OVERHEAD_SIZE) ||
        (frame_length > UART3_PROTOCOL_MAX_FRAME_SIZE))
    {
        return HAL_ERROR;
    }

    __disable_irq();
    if (s_tx_queue_count >= UART3_PROTOCOL_TX_QUEUE_DEPTH)
    {
        __enable_irq();
        return HAL_BUSY;
    }

    enqueue_index = s_tx_queue_tail;
    memcpy(s_tx_queue[enqueue_index].data, frame_data, frame_length);
    s_tx_queue[enqueue_index].length = frame_length;
    s_tx_queue_tail = (uint8_t)((s_tx_queue_tail + 1U) % UART3_PROTOCOL_TX_QUEUE_DEPTH);
    s_tx_queue_count++;
    __enable_irq();

    return UART3_Protocol_StartNextTx();
}

static HAL_StatusTypeDef UART3_Protocol_StartNextTx(void)
{
    uint8_t tx_index = 0U;
    uint8_t tx_length = 0U;
    uint8_t start_tx = 0U;
    HAL_StatusTypeDef hal_status = HAL_OK;

    __disable_irq();
    if ((s_tx_dma_busy == 0U) && (s_tx_queue_count > 0U))
    {
        tx_index = s_tx_queue_head;
        tx_length = s_tx_queue[tx_index].length;
        s_tx_dma_busy = 1U;
        start_tx = 1U;
    }
    __enable_irq();

    if (start_tx == 0U)
    {
        return HAL_OK;
    }

    hal_status = HAL_UART_Transmit_DMA(&huart3, s_tx_queue[tx_index].data, tx_length);
    if (hal_status != HAL_OK)
    {
        __disable_irq();
        s_tx_dma_busy = 0U;
        __enable_irq();
        s_protocol_stats.tx_start_error_count++;
    }

    return hal_status;
}
