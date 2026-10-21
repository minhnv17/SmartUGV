#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

#include "MavlinkStreamer.h"
#include "common/mavlink.h"

#include "uORB.h"
#include "Imu.h"
#include "MotorController.h"

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
#define CONTROL_TIMEOUT_MS 500u /* Hết 500ms không có lệnh -> Failsafe dừng xe */
#define JOYSTICK_DEADBAND 50    /* Ngưỡng chết joystick tránh trôi */

static void MavlinkHeartbeatStreamer(void);
static void MAVLinkImuStreamer(void);

static TaskHandle_t MavlinkStreamerTaskHandle = NULL;
static TaskHandle_t MavlinkGcsRequestTaskHandler = NULL;
static SemaphoreHandle_t xTxDmaSemaphore = NULL;
static QueueHandle_t xMavlinkRxQueue = NULL;

static uint16_t ImuSensorFd = -1;
static uint8_t rx_dma_buffer[UART_RX_BUF_SIZE];

/* Watchdog failsafe & Trạng thái Arm */
static volatile uint32_t s_last_control_time_ms = 0;
static volatile bool s_motor_is_active = false;
static volatile bool s_is_armed = false;

static MavlinkStream_t g_mavlink_streams[] = {
    // { Rate_Hz, Last_sent, Send_function }
    {.rate_hz = 1, .last_sent_ms = 0, .send = MavlinkHeartbeatStreamer}, // 1 Hz Heartbeat
    {.rate_hz = 50, .last_sent_ms = 0, .send = MAVLinkImuStreamer},      // 50 Hz IMU
};

#define NUM_STREAMS (sizeof(g_mavlink_streams) / sizeof(MavlinkStream_t))

extern UART_HandleTypeDef huart1;

/* -----------------------------------------------------------------------
 * Helper gửi gói tin MAVLink an toàn qua UART DMA
 * ----------------------------------------------------------------------- */
static bool Mavlink_Transmit(mavlink_message_t *msg)
{
    static uint8_t s_tx_buf[MAVLINK_MAX_PACKET_LEN];

    if (xSemaphoreTake(xTxDmaSemaphore, pdMS_TO_TICKS(50)) == pdTRUE)
    {
        uint16_t len = mavlink_msg_to_send_buffer(s_tx_buf, msg);
        if (HAL_UART_Transmit_DMA(&huart1, s_tx_buf, len) == HAL_OK)
        {
            return true;
        }
        else
        {
            xSemaphoreGive(xTxDmaSemaphore);
        }
    }
    return false;
}

/* -----------------------------------------------------------------------
 * Xử lý lệnh vận tốc vi sai (Differential Drive / Skid-steer)
 *   throttle: [-1000, 1000] (> 0: tiến, < 0: lùi)
 *   steer:    [-1000, 1000] (> 0: rẽ phải, < 0: rẽ trái)
 * ----------------------------------------------------------------------- */
static void MAVLink_Process_Drive_Command(int16_t throttle, int16_t steer)
{
    /* Nếu xe đang DISARM -> Chặn toàn bộ lệnh, dừng motor ngay */
    if (!s_is_armed)
    {
        MotorController_StopAllMotors();
        s_motor_is_active = false;
        return;
    }

    /* Lọc vùng chết cần gạt */
    if (abs(throttle) < JOYSTICK_DEADBAND) throttle = 0;
    if (abs(steer) < JOYSTICK_DEADBAND) steer = 0;

    if (throttle == 0 && steer == 0)
    {
        MotorController_StopAllMotors();
        s_motor_is_active = false;
        return;
    }

    /* Thuật toán vi sai 2 bánh:
     * MOTOR_1 = Bánh trái, MOTOR_2 = Bánh phải
     * Tiến: cả 2 cùng dương (FORWARD)
     * Lùi: cả 2 cùng âm (REVERSE)
     * Rẽ phải: bánh trái nhanh hơn (tăng), bánh phải chậm/lùi (giảm)
     * Rẽ trái: bánh trái giảm, bánh phải tăng
     */
    int32_t left_cmd  = (int32_t)throttle + (int32_t)steer;
    int32_t right_cmd = (int32_t)throttle - (int32_t)steer;

    /* Clamp vào dải [-MAX_MOTOR_SPEED, MAX_MOTOR_SPEED] */
    if (left_cmd > (int32_t)MAX_MOTOR_SPEED)   left_cmd = (int32_t)MAX_MOTOR_SPEED;
    if (left_cmd < -(int32_t)MAX_MOTOR_SPEED)  left_cmd = -(int32_t)MAX_MOTOR_SPEED;
    if (right_cmd > (int32_t)MAX_MOTOR_SPEED)  right_cmd = (int32_t)MAX_MOTOR_SPEED;
    if (right_cmd < -(int32_t)MAX_MOTOR_SPEED) right_cmd = -(int32_t)MAX_MOTOR_SPEED;

    /* Điều khiển Bánh Trái (MOTOR_1) */
    if (left_cmd > 0)
    {
        MotorController_SetMotorSpeed(MOTOR_1, (uint16_t)left_cmd, FORWARD);
    }
    else if (left_cmd < 0)
    {
        MotorController_SetMotorSpeed(MOTOR_1, (uint16_t)(-left_cmd), REVERSE);
    }
    else
    {
        MotorController_SetMotorSpeed(MOTOR_1, 0, FORWARD);
    }

    /* Điều khiển Bánh Phải (MOTOR_2) */
    if (right_cmd > 0)
    {
        MotorController_SetMotorSpeed(MOTOR_2, (uint16_t)right_cmd, FORWARD);
    }
    else if (right_cmd < 0)
    {
        MotorController_SetMotorSpeed(MOTOR_2, (uint16_t)(-right_cmd), REVERSE);
    }
    else
    {
        MotorController_SetMotorSpeed(MOTOR_2, 0, FORWARD);
    }

    s_last_control_time_ms = HAL_GetTick();
    s_motor_is_active = true;
}

/* -----------------------------------------------------------------------
 * Hàm xử lý gói tin MAVLink nhận được từ GCS / Điều khiển
 * ----------------------------------------------------------------------- */
static void MAVLink_Handle_Incoming_Message(mavlink_message_t *msg)
{
    switch (msg->msgid)
    {
    /* 1. Nhận lệnh điều khiển Joystick / Manual Control */
    case MAVLINK_MSG_ID_MANUAL_CONTROL:
    {
        mavlink_manual_control_t manual_ctrl;
        mavlink_msg_manual_control_decode(msg, &manual_ctrl);

        /* x: [-1000, 1000] - Tiến/Lùi
         * r: [-1000, 1000] - Lái (Yaw)
         * y: [-1000, 1000] - Roll (fallback nếu r == 0) */
        int16_t throttle = manual_ctrl.x;
        int16_t steer = (manual_ctrl.r != 0) ? manual_ctrl.r : manual_ctrl.y;

        MAVLink_Process_Drive_Command(throttle, steer);
        break;
    }

    /* 2. Nhận kênh RC Override (hỗ trợ điều khiển qua RC channels / Python) */
    case MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE:
    {
        mavlink_rc_channels_override_t rc;
        mavlink_msg_rc_channels_override_decode(msg, &rc);

        int16_t throttle = 0;
        int16_t steer = 0;

        /* Kênh 3 (hoặc 2): Throttle (1000..2000 us, 1500 us là giữa) */
        if (rc.chan3_raw >= 1000 && rc.chan3_raw <= 2000)
        {
            throttle = (int16_t)((int32_t)(rc.chan3_raw - 1500) * 2);
        }
        else if (rc.chan2_raw >= 1000 && rc.chan2_raw <= 2000)
        {
            throttle = (int16_t)((int32_t)(rc.chan2_raw - 1500) * 2);
        }

        /* Kênh 1: Steering (1000..2000 us, 1500 us là giữa) */
        if (rc.chan1_raw >= 1000 && rc.chan1_raw <= 2000)
        {
            steer = (int16_t)((int32_t)(rc.chan1_raw - 1500) * 2);
        }

        MAVLink_Process_Drive_Command(throttle, steer);
        break;
    }

    /* 3. QGC hỏi danh sách Parameter */
    case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
    {
        mavlink_message_t reply_msg;
        mavlink_msg_param_value_pack(
            1, MAV_COMP_ID_AUTOPILOT1, &reply_msg,
            "DUMMY_PARAM",
            0.0f,
            MAV_PARAM_TYPE_REAL32,
            0, // param_count = 0
            0  // param_index
        );
        Mavlink_Transmit(&reply_msg);
        break;
    }

    /* 4. QGC hỏi Parameter lẻ */
    case MAVLINK_MSG_ID_PARAM_REQUEST_READ:
    {
        break;
    }

    /* 5. QGC gửi lệnh Command Long (Arm/Disarm, Motor Test...) */
    case MAVLINK_MSG_ID_COMMAND_LONG:
    {
        mavlink_command_long_t cmd;
        mavlink_msg_command_long_decode(msg, &cmd);

        uint8_t result = MAV_RESULT_ACCEPTED;

        if (cmd.command == MAV_CMD_COMPONENT_ARM_DISARM)
        {
            if ((int)cmd.param1 == 1)
            {
                s_is_armed = true;
            }
            else if ((int)cmd.param1 == 0) // Disarm -> Dừng khẩn cấp motor
            {
                s_is_armed = false;
                MotorController_StopAllMotors();
                s_motor_is_active = false;
            }
        }

        mavlink_message_t ack_msg;
        mavlink_msg_command_ack_pack(
            1, MAV_COMP_ID_AUTOPILOT1, &ack_msg,
            cmd.command,
            result,
            255, 0, msg->sysid, msg->compid);

        Mavlink_Transmit(&ack_msg);
        break;
    }

    default:
        break;
    }
}

/* -----------------------------------------------------------------------
 * Stream Heartbeat (1Hz)
 * ----------------------------------------------------------------------- */
static void MavlinkHeartbeatStreamer(void)
{
    mavlink_message_t msg;

    uint8_t base_mode = MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
    if (s_is_armed)
    {
        base_mode |= MAV_MODE_FLAG_SAFETY_ARMED;
    }

    mavlink_msg_heartbeat_pack(
        1,                      // System ID: 1
        MAV_COMP_ID_AUTOPILOT1, // Component ID
        &msg,
        MAV_TYPE_GROUND_ROVER,  // Định danh là UGV / Xe tự hành mặt đất
        MAV_AUTOPILOT_GENERIC,  // Generic Autopilot
        base_mode,
        0,
        s_is_armed ? MAV_STATE_ACTIVE : MAV_STATE_STANDBY
    );

    Mavlink_Transmit(&msg);
}

/* -----------------------------------------------------------------------
 * Stream IMU (50Hz)
 * ----------------------------------------------------------------------- */
static void MAVLinkImuStreamer(void)
{
    mavlink_raw_imu_t imu_data;
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

    Mavlink_Transmit(&msg);
}

/* -----------------------------------------------------------------------
 * Task nhận và xử lý lệnh từ MAVLink GCS / Controller
 * ----------------------------------------------------------------------- */
static void MAVLinkGcsRequestTask(void *args)
{
    mavlink_message_t rx_msg;

    while (1)
    {
        /* Chờ gói tin trong queue tối đa 50ms (không busy-loop) */
        if (xQueueReceive(xMavlinkRxQueue, &rx_msg, pdMS_TO_TICKS(50)) == pdTRUE)
        {
            MAVLink_Handle_Incoming_Message(&rx_msg);
        }

        /* Failsafe Watchdog: Nếu motor đang chạy mà mất tín hiệu điều khiển > 500ms -> Dừng an toàn */
        if (s_motor_is_active && ((HAL_GetTick() - s_last_control_time_ms) > CONTROL_TIMEOUT_MS))
        {
            MotorController_StopAllMotors();
            s_motor_is_active = false;
        }
    }
}

/* -----------------------------------------------------------------------
 * Task định kỳ gửi dữ liệu Telemetry (Heartbeat, IMU)
 * ----------------------------------------------------------------------- */
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
                    g_mavlink_streams[i].send();
                }
            }
        }

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(10));
    }
}

/* -----------------------------------------------------------------------
 * Khởi tạo MAVLink Streamer
 * ----------------------------------------------------------------------- */
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

/* -----------------------------------------------------------------------
 * Callbacks UART DMA
 * ----------------------------------------------------------------------- */
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
    static mavlink_message_t rx_msg;
    static mavlink_status_t rx_status;

    if (huart->Instance == USART1)
    {
        for (uint16_t i = 0; i < Size; i++)
        {
            uint8_t byte = rx_dma_buffer[i];

            /* MAVLink parser tự nhận diện frame v1 (0xFE) và v2 (0xFD) */
            if (mavlink_parse_char(MAVLINK_COMM_0, byte, &rx_msg, &rx_status))
            {
                xQueueSendFromISR(xMavlinkRxQueue, &rx_msg, NULL);
            }
        }

        /* Bật lại DMA chờ khung truyền tiếp theo */
        HAL_UARTEx_ReceiveToIdle_DMA(huart, rx_dma_buffer, UART_RX_BUF_SIZE);
    }
}
