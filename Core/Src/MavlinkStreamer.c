#include <stdint.h>

#include "MavlinkStreamer.h"
#include "common/mavlink.h"

#include "uORB.h"
#include "Imu.h"

#include "stm32f1xx_hal.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"

typedef void (*mavlink_send_cb_t)(void);

typedef struct
{
    uint32_t rate_hz;       // Tần số muốn gửi (Hz)
    uint32_t last_sent_ms;  // Thời điểm gửi lần cuối
    mavlink_send_cb_t send; // Hàm phụ trách gửi gói tin đó
} MavlinkStream_t;

#define UART_RX_BUF_SIZE 512
static void MavlinkHeartbeatStreamer(void);
static void MAVLinkImuStreamer(void);

static TaskHandle_t MavlinkStreamerTaskHandle, MavlinkGcsRequestTaskHandler = NULL;
static SemaphoreHandle_t xTxDmaSemaphore;
static QueueHandle_t xMavlinkRxQueue;

static uint16_t ImuSensorFd = -1;

static uint8_t rx_dma_buffer[UART_RX_BUF_SIZE];

static MavlinkStream_t g_mavlink_streams[] = {
    // { Rate_Hz, Interval_ms, Last_sent, Send_function }
    {.rate_hz = 1, .last_sent_ms = 0, .send = MavlinkHeartbeatStreamer}, // 1 Hz
    {.rate_hz = 50, .last_sent_ms = 0, .send = MAVLinkImuStreamer},      // 50 Hz
};

#define NUM_STREAMS (sizeof(g_mavlink_streams) / sizeof(MavlinkStream_t))

extern UART_HandleTypeDef huart1;

// Hàm xử lý gói tin MAVLink nhận được từ QGC
static void MAVLink_Handle_Incoming_Message(mavlink_message_t *msg)
{
    switch (msg->msgid)
    {

    // 1. QGC hỏi danh sách Parameter -> PHẢI PHẢN HỒI NGAY
    case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
    {
        mavlink_message_t reply_msg;
        uint8_t tx_buf[MAVLINK_MAX_PACKET_LEN];

        // Báo với QGC là hệ thống có 0 parameter
        mavlink_msg_param_value_pack(
            1, MAV_COMP_ID_AUTOPILOT1, &reply_msg,
            "DUMMY_PARAM",
            0.0f,
            MAV_PARAM_TYPE_REAL32,
            0, // param_count = 0
            0  // param_index
        );

        uint16_t len = mavlink_msg_to_send_buffer(tx_buf, &reply_msg);
        HAL_UART_Transmit_DMA(&huart1, tx_buf, len);
        break;
    }

    // 2. QGC hỏi Parameter lẻ
    case MAVLINK_MSG_ID_PARAM_REQUEST_READ:
    {
        // Xử lý đọc param cụ thể nếu sau này bạn làm
        break;
    }

    // 3. QGC gửi lệnh (Arm/Disarm, Calib...) -> Bắt buộc ACK lại
    case MAVLINK_MSG_ID_COMMAND_LONG:
    {
        mavlink_command_long_t cmd;
        mavlink_msg_command_long_decode(msg, &cmd);

        // Trả ACK lại cho QGC biết đã nhận lệnh
        mavlink_message_t ack_msg;
        uint8_t tx_buf[MAVLINK_MAX_PACKET_LEN];

        mavlink_msg_command_ack_pack(
            1, MAV_COMP_ID_AUTOPILOT1, &ack_msg,
            cmd.command,
            MAV_RESULT_ACCEPTED, // Báo nhận lệnh thành công
            255, 0, msg->sysid, msg->compid);

        uint16_t len = mavlink_msg_to_send_buffer(tx_buf, &ack_msg);
        HAL_UART_Transmit_DMA(&huart1, tx_buf, len);
        break;
    }

    default:
        break;
    }
}

static void MavlinkHeartbeatStreamer(void)
{
    mavlink_message_t msg;
    static uint8_t tx_buf[MAVLINK_MAX_PACKET_LEN];

    mavlink_msg_heartbeat_pack(
        1,                      // System ID (ID thiết bị của bạn, VD: 1)
        MAV_COMP_ID_AUTOPILOT1, // Component ID (VD: 200)
        &msg,                   // Con trỏ tới struct message
        MAV_TYPE_QUADROTOR,     // Loại thiết bị
        MAV_AUTOPILOT_PX4,      // Loại Autopilot
        MAV_MODE_PREFLIGHT,     // Chế độ hoạt động
        0,                      // Custom mode
        MAV_STATE_STANDBY       // Trạng thái
    );

    uint16_t len = mavlink_msg_to_send_buffer(tx_buf, &msg);

    if (xSemaphoreTake(xTxDmaSemaphore, pdMS_TO_TICKS(20)) == pdTRUE)
    {
        if (HAL_UART_Transmit_DMA(&huart1, tx_buf, len) != HAL_OK)
        {
            xSemaphoreGive(xTxDmaSemaphore);
        }
    }
}

static void MAVLinkImuStreamer(void)
{
    mavlink_raw_imu_t imu_data;
    uint8_t tx_buf[MAVLINK_MAX_PACKET_LEN];
    mavlink_message_t msg;

    uORB_CopyData(ImuSensorFd, &imu_data);

    mavlink_msg_raw_imu_pack(1, MAV_COMP_ID_IMU, &msg,
                             imu_data.time_usec,
                             imu_data.xacc,
                             imu_data.yacc,
                             imu_data.zacc,
                             imu_data.xgyro,
                             imu_data.ygyro,
                             imu_data.zgyro,
                             imu_data.xmag,
                             imu_data.ymag,
                             imu_data.zmag,
                             imu_data.id,
                             imu_data.temperature);

    uint16_t len = mavlink_msg_to_send_buffer(tx_buf, &msg);

    if (xSemaphoreTake(xTxDmaSemaphore, pdMS_TO_TICKS(20)) == pdTRUE)
    {
        if (HAL_UART_Transmit_DMA(&huart1, tx_buf, len) != HAL_OK)
        {
            xSemaphoreGive(xTxDmaSemaphore);
        }
    }
}

static void MAVLinkGcsRequestTask(void *args)
{
    mavlink_message_t rx_msg;
    while (1)
    {
        if (xQueueReceive(xMavlinkRxQueue, &rx_msg, 0) == pdTRUE)
        {
            if (xSemaphoreTake(xTxDmaSemaphore, portMAX_DELAY) == pdTRUE)
            {
                MAVLink_Handle_Incoming_Message(&rx_msg);
                xSemaphoreGive(xTxDmaSemaphore);
            }
        }
    }
}

static void MAVLinkStreamerTask(void *args)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (1)
    {
        uint32_t now = HAL_GetTick();

        for (uint8_t i = 0; i < NUM_STREAMS; i++)
        {
            if (g_mavlink_streams[i].rate_hz > 0)
            {
                if ((now - g_mavlink_streams[i].last_sent_ms) >= (1000 / g_mavlink_streams[i].rate_hz))
                {
                    g_mavlink_streams[i].last_sent_ms = now;
                    g_mavlink_streams[i].send(); // Gọi hàm phát tương ứng
                }
            }
        }

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(10));
    }
}

uint8_t MavlinkStreamer_Init(void)
{
    ImuSensorFd = uORB_Subscribe("sensor/imu_data");

    xTxDmaSemaphore = xSemaphoreCreateBinary();
    xSemaphoreGive(xTxDmaSemaphore);

    xMavlinkRxQueue = xQueueCreate(15, sizeof(mavlink_message_t));

    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rx_dma_buffer, UART_RX_BUF_SIZE);

    xTaskCreate(MAVLinkStreamerTask, "MavlinkStreamer", configMINIMAL_STACK_SIZE * 3, NULL, 1, &MavlinkStreamerTaskHandle);
    xTaskCreate(MAVLinkGcsRequestTask, "MavlinkResponse", configMINIMAL_STACK_SIZE * 3, NULL, 1, &MavlinkGcsRequestTaskHandler);
    return 0;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;

        xSemaphoreGiveFromISR(xTxDmaSemaphore, &xHigherPriorityTaskWoken);

        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    mavlink_message_t rx_msg;
    mavlink_status_t rx_status;

    if (huart->Instance == USART1)
    {
        for (uint16_t i = 0; i < Size; i++)
        {
            if (mavlink_parse_char(MAVLINK_COMM_0, rx_dma_buffer[i], &rx_msg, &rx_status))
            {
                // Parse thành công 1 message MAVLink
                xQueueSendFromISR(xMavlinkRxQueue, &rx_msg, NULL);
            }
        }

        // Bật lại DMA chờ khung truyền tiếp theo từ QGC
        HAL_UARTEx_ReceiveToIdle_DMA(huart, rx_dma_buffer, UART_RX_BUF_SIZE);
    }
}
