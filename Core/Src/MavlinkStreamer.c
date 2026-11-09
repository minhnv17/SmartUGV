#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
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

#define UART_RX_BUF_SIZE 512
#define CONTROL_TIMEOUT_MS 500u
#define CONTROL_MAX_QUEUE_AGE_MS 150u
#define JOYSTICK_DEADBAND 50
typedef struct { mavlink_message_t message; uint32_t received_ms; } ReceivedMessage_t;
static TaskHandle_t MavlinkStreamerTaskHandle, MavlinkGcsRequestTaskHandler;
static SemaphoreHandle_t xTxDmaSemaphore;
static QueueHandle_t xMavlinkRxQueue;
static uint16_t ImuSensorFd;
static uint8_t rx_dma_buffer[UART_RX_BUF_SIZE];
static mavlink_message_t s_rx_message;
static mavlink_status_t s_rx_status, s_tx_status;
static volatile bool s_rx_fault, s_rx_restart;
static uint32_t s_last_control_time_ms;
static volatile bool s_is_armed;
extern UART_HandleTypeDef huart1;

static bool Mavlink_Transmit(mavlink_message_t *message)
{
    static uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    if (xSemaphoreTake(xTxDmaSemaphore,pdMS_TO_TICKS(50)) != pdTRUE) return false;
    const mavlink_msg_entry_t *entry=mavlink_get_msg_entry(message->msgid);
    if (!entry) { xSemaphoreGive(xTxDmaSemaphore); return false; }
    // Serialization and sequence allocation are protected for both TX tasks.
    mavlink_finalize_message_buffer(message,1,message->compid,&s_tx_status,
        entry->min_msg_len,message->len,entry->crc_extra);
    uint16_t length=mavlink_msg_to_send_buffer(buffer,message);
    if (HAL_UART_Transmit_DMA(&huart1,buffer,length)==HAL_OK) return true;
    xSemaphoreGive(xTxDmaSemaphore);
    return false;
}

static void Disarm(void)
{
    s_is_armed=false;
    MotorController_StopAllMotors();
}

static bool ControllerSource(const mavlink_message_t *message)
{
    return message->sysid==1 && message->compid==MAV_COMP_ID_ONBOARD_COMPUTER;
}

static void MavlinkHeartbeatStreamer(void)
{
    mavlink_message_t message;
    mavlink_status_t status={0};
    mavlink_msg_heartbeat_pack_status(1,MAV_COMP_ID_AUTOPILOT1,&status,&message,
        MAV_TYPE_GROUND_ROVER,MAV_AUTOPILOT_GENERIC,
        MAV_MODE_FLAG_MANUAL_INPUT_ENABLED | (s_is_armed ? MAV_MODE_FLAG_SAFETY_ARMED : 0),
        0,s_is_armed ? MAV_STATE_ACTIVE : MAV_STATE_STANDBY);
    Mavlink_Transmit(&message);
}

static void ControlVersionStreamer(void)
{
    static const char name[10]="CTRL_VER";
    mavlink_message_t message;
    mavlink_status_t status={0};
    mavlink_msg_named_value_int_pack_status(1,MAV_COMP_ID_AUTOPILOT1,&status,&message,HAL_GetTick(),name,1);
    Mavlink_Transmit(&message);
}

static void MAVLinkImuStreamer(void)
{
    mavlink_raw_imu_t imu={0};
    if (uORB_CopyData(ImuSensorFd,&imu)<0) return;
    mavlink_message_t message;
    mavlink_status_t status={0};
    mavlink_msg_raw_imu_pack_status(1,MAV_COMP_ID_IMU,&status,&message,imu.time_usec,
        imu.xacc,imu.yacc,imu.zacc,imu.xgyro,imu.ygyro,imu.zgyro,imu.xmag,imu.ymag,imu.zmag,
        imu.id,imu.temperature);
    Mavlink_Transmit(&message);
}

static void MAVLink_Process_Drive_Command(int16_t throttle, int16_t steer)
{
    if (throttle < -1000 || throttle > 1000 || steer < -1000 || steer > 1000) { Disarm(); return; }
    if (!s_is_armed) { MotorController_StopAllMotors(); return; }
    if (abs(throttle)<JOYSTICK_DEADBAND) throttle=0;
    if (abs(steer)<JOYSTICK_DEADBAND) steer=0;
    s_last_control_time_ms=HAL_GetTick(); // Neutral also maintains the short ARM handshake.
    if (!throttle && !steer) { MotorController_StopAllMotors(); return; }
    int32_t values[2]={(int32_t)throttle+steer,(int32_t)throttle-steer};
    for (unsigned i=0;i<2;++i) {
        if (values[i]>(int32_t)MAX_MOTOR_SPEED) values[i]=MAX_MOTOR_SPEED;
        if (values[i]<-(int32_t)MAX_MOTOR_SPEED) values[i]=-(int32_t)MAX_MOTOR_SPEED;
        MotorController_SetMotorSpeed((MotorID_t)i,(uint16_t)abs((int)values[i]),values[i]<0 ? REVERSE : FORWARD);
    }
}

static void MAVLink_Handle_Incoming_Message(const mavlink_message_t *message, uint32_t received_ms)
{
    if (!ControllerSource(message)) return;
    bool stale=HAL_GetTick()-received_ms>CONTROL_MAX_QUEUE_AGE_MS;
    if (message->msgid==MAVLINK_MSG_ID_MANUAL_CONTROL) {
        mavlink_manual_control_t control;
        mavlink_msg_manual_control_decode(message,&control);
        if (control.target!=1) return;
        // Deliberately use x/r only; no ambiguous y fallback or invalid-axis sentinels.
        if (stale && (control.x || control.r)) { Disarm(); return; }
        MAVLink_Process_Drive_Command(control.x,control.r);
    } else if (message->msgid==MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE) {
        mavlink_rc_channels_override_t rc;
        mavlink_msg_rc_channels_override_decode(message,&rc);
        if (rc.target_system!=1 || rc.target_component!=MAV_COMP_ID_AUTOPILOT1) return;
        if (stale || rc.chan1_raw<1000 || rc.chan1_raw>2000 || rc.chan3_raw<1000 || rc.chan3_raw>2000) {
            Disarm(); return;
        }
        MAVLink_Process_Drive_Command(((int32_t)rc.chan3_raw-1500)*2,((int32_t)rc.chan1_raw-1500)*2);
    } else if (message->msgid==MAVLINK_MSG_ID_COMMAND_LONG) {
        mavlink_command_long_t command;
        mavlink_msg_command_long_decode(message,&command);
        if (command.target_system!=1 || command.target_component!=MAV_COMP_ID_AUTOPILOT1) return;
        uint8_t result=MAV_RESULT_UNSUPPORTED;
        if (command.command==MAV_CMD_COMPONENT_ARM_DISARM) {
            if (!isfinite(command.param1) || (command.param1!=0.0f && command.param1!=1.0f) || command.param2!=0.0f)
                result=MAV_RESULT_DENIED;
            else if (command.param1==0.0f) { Disarm(); result=MAV_RESULT_ACCEPTED; }
            else if (stale || s_rx_fault) result=MAV_RESULT_TEMPORARILY_REJECTED;
            else {
                if (!s_is_armed) { MotorController_StopAllMotors(); s_last_control_time_ms=HAL_GetTick(); }
                s_is_armed=true;
                result=MAV_RESULT_ACCEPTED;
            }
        }
        mavlink_message_t ack;
        mavlink_status_t status={0};
        mavlink_msg_command_ack_pack_status(1,MAV_COMP_ID_AUTOPILOT1,&status,&ack,command.command,
            result,255,0,message->sysid,message->compid);
        Mavlink_Transmit(&ack);
        MavlinkHeartbeatStreamer();
    }
}

static void CheckControlTimeout(void)
{
    if (s_is_armed && HAL_GetTick()-s_last_control_time_ms>=CONTROL_TIMEOUT_MS) Disarm();
}

static bool StartReceive(void)
{
    if (HAL_UARTEx_ReceiveToIdle_DMA(&huart1,rx_dma_buffer,UART_RX_BUF_SIZE)!=HAL_OK) return false;
    // Normal DMA: HT does not stop reception. Only process IDLE/TC, once.
    __HAL_DMA_DISABLE_IT(huart1.hdmarx,DMA_IT_HT);
    return true;
}

static void MAVLinkGcsRequestTask(void *args)
{
    ReceivedMessage_t received;
    for (;;) {
        if (s_rx_fault) {
            Disarm();
            taskENTER_CRITICAL();
            xQueueReset(xMavlinkRxQueue);
            memset(&s_rx_status,0,sizeof(s_rx_status));
            s_rx_fault=false;
            taskEXIT_CRITICAL();
        }
        if (s_rx_restart) {
            HAL_UART_AbortReceive(&huart1);
            taskENTER_CRITICAL();
            memset(&s_rx_status,0,sizeof(s_rx_status));
            taskEXIT_CRITICAL();
            s_rx_restart=!StartReceive();
        }
        if (xQueueReceive(xMavlinkRxQueue,&received,pdMS_TO_TICKS(10))==pdTRUE)
            MAVLink_Handle_Incoming_Message(&received.message,received.received_ms);
        CheckControlTimeout();
    }
}

static void MAVLinkStreamerTask(void *args)
{
    TickType_t last=xTaskGetTickCount();
    uint32_t heartbeat_ms=0,imu_ms=0;
    for (;;) {
        uint32_t now=HAL_GetTick();
        if (now-heartbeat_ms>=1000) { heartbeat_ms=now; MavlinkHeartbeatStreamer(); ControlVersionStreamer(); }
        if (now-imu_ms>=20) { imu_ms=now; MAVLinkImuStreamer(); }
        vTaskDelayUntil(&last,pdMS_TO_TICKS(10));
    }
}

uint8_t MavlinkStreamer_Init(void)
{
    ImuSensorFd=uORB_Subscribe("sensor/imu_data");
    xTxDmaSemaphore=xSemaphoreCreateBinary();
    xMavlinkRxQueue=xQueueCreate(8,sizeof(ReceivedMessage_t));
    if (!xTxDmaSemaphore || !xMavlinkRxQueue || ImuSensorFd==(uint16_t)-1) return 1;
    xSemaphoreGive(xTxDmaSemaphore);
    if (!StartReceive()) return 1;
    if (xTaskCreate(MAVLinkStreamerTask,"MavlinkStreamer",configMINIMAL_STACK_SIZE*3,NULL,1,
        &MavlinkStreamerTaskHandle)!=pdPASS) return 1;
    if (xTaskCreate(MAVLinkGcsRequestTask,"MavlinkResponse",configMINIMAL_STACK_SIZE*4,NULL,3,
        &MavlinkGcsRequestTaskHandler)!=pdPASS) return 1;
    return 0;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *uart)
{
    if (uart->Instance==USART1 && xTxDmaSemaphore) {
        BaseType_t wake=pdFALSE;
        xSemaphoreGiveFromISR(xTxDmaSemaphore,&wake);
        portYIELD_FROM_ISR(wake);
    }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *uart, uint16_t size)
{
    if (uart->Instance!=USART1 || size>UART_RX_BUF_SIZE ||
        HAL_UARTEx_GetRxEventType(uart)==HAL_UART_RXEVENT_HT) return;
    BaseType_t wake=pdFALSE;
    for (uint16_t i=0;i<size;++i) {
        ReceivedMessage_t received;
        if (mavlink_frame_char_buffer(&s_rx_message,&s_rx_status,rx_dma_buffer[i],&received.message,NULL)==MAVLINK_FRAMING_OK) {
            received.received_ms=HAL_GetTick();
            if (xQueueSendFromISR(xMavlinkRxQueue,&received,&wake)!=pdTRUE) s_rx_fault=true;
        }
    }
    if (!StartReceive()) { s_rx_fault=true; s_rx_restart=true; }
    portYIELD_FROM_ISR(wake);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *uart)
{
    if (uart->Instance==USART1) { s_rx_fault=true; s_rx_restart=true; }
}
