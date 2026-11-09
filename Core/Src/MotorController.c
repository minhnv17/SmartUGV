#include <stdbool.h>
#include <stdint.h>
#include "main.h"
#include "MotorController.h"
#include "FreeRTOS.h"
#include "task.h"

extern TIM_HandleTypeDef htim2;
typedef enum { MOTOR_IDLE, MOTOR_KICKSTART, MOTOR_RUNNING, MOTOR_REVERSING } MotorState_t;
typedef struct {
    MotorState_t state;
    Direction_t direction, target_direction;
    uint16_t target_speed;
    uint32_t phase_ms;
} MotorCtrl_t;
static MotorCtrl_t g_motors[2];
static TaskHandle_t motorControllerTaskHandler;

static void Motor_SetPWM(MotorID_t id, uint16_t speed)
{
    __HAL_TIM_SET_COMPARE(&htim2, id == MOTOR_1 ? TIM_CHANNEL_1 : TIM_CHANNEL_2, speed);
}

static void Motor_Coast(MotorID_t id)
{
    if (id == MOTOR_1) {
        HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_RESET);
    } else {
        HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, GPIO_PIN_RESET);
    }
}

static void Motor_ApplyDirection(MotorID_t id, Direction_t direction)
{
    if (id == MOTOR_1) {
        HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, direction == FORWARD ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, direction == FORWARD ? GPIO_PIN_RESET : GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, direction == FORWARD ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, direction == FORWARD ? GPIO_PIN_RESET : GPIO_PIN_SET);
    }
}

// Caller holds the critical section. A stop cancels every queued target and
// acts directly on hardware, including during kickstart/reversal.
static void Motor_StopLocked(MotorID_t id)
{
    Motor_SetPWM(id, 0);
    Motor_Coast(id);
    g_motors[id].target_speed = 0;
    g_motors[id].state = MOTOR_IDLE;
}

static void Motor_Tick(MotorID_t id)
{
    taskENTER_CRITICAL();
    MotorCtrl_t *motor = &g_motors[id];
    uint32_t now = HAL_GetTick();
    if (!motor->target_speed) {
        Motor_StopLocked(id);
    } else if (motor->state != MOTOR_IDLE && motor->state != MOTOR_REVERSING &&
               motor->direction != motor->target_direction) {
        Motor_SetPWM(id, 0);
        Motor_Coast(id);
        motor->state = MOTOR_REVERSING;
        motor->phase_ms = now;
    } else if (motor->state == MOTOR_IDLE ||
              (motor->state == MOTOR_REVERSING && now - motor->phase_ms >= 10)) {
        motor->direction = motor->target_direction;
        Motor_ApplyDirection(id, motor->direction);
        Motor_SetPWM(id, MAX_MOTOR_SPEED);
        motor->state = MOTOR_KICKSTART;
        motor->phase_ms = now;
    } else if (motor->state == MOTOR_KICKSTART && now - motor->phase_ms >= KICKSTART_DURATION_MS) {
        Motor_SetPWM(id, motor->target_speed);
        motor->state = MOTOR_RUNNING;
    } else if (motor->state == MOTOR_RUNNING) {
        Motor_SetPWM(id, motor->target_speed);
    }
    taskEXIT_CRITICAL();
}

static void MotorControllerTask(void *args)
{
    TickType_t last = xTaskGetTickCount();
    for (;;) {
        Motor_Tick(MOTOR_1);
        Motor_Tick(MOTOR_2);
        vTaskDelayUntil(&last, pdMS_TO_TICKS(10));
    }
}

void MotorController_Init(void)
{
    for (unsigned i=0; i<2; ++i) g_motors[i] = (MotorCtrl_t){.direction=FORWARD,.target_direction=FORWARD};
    Motor_Coast(MOTOR_1);
    Motor_Coast(MOTOR_2);
    if (HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1) != HAL_OK ||
        HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2) != HAL_OK) Error_Handler();
    MotorController_StopAllMotors();
    if (xTaskCreate(MotorControllerTask,"MotorCtrl",configMINIMAL_STACK_SIZE*2,NULL,5,
                    &motorControllerTaskHandler) != pdPASS) Error_Handler();
}

void MotorController_SetMotorSpeed(MotorID_t id, uint16_t speed, Direction_t direction)
{
    if ((unsigned)id >= 2 || (unsigned)direction > REVERSE) return;
    if (speed > MAX_MOTOR_SPEED) speed = MAX_MOTOR_SPEED;
    if (speed && speed < MOTOR_DEAD_ZONE) speed = MOTOR_DEAD_ZONE;
    taskENTER_CRITICAL();
    if (!speed) Motor_StopLocked(id);
    else {
        g_motors[id].target_speed = speed;
        g_motors[id].target_direction = direction;
    }
    taskEXIT_CRITICAL();
}

void MotorController_StopAllMotors(void)
{
    taskENTER_CRITICAL();
    Motor_StopLocked(MOTOR_1);
    Motor_StopLocked(MOTOR_2);
    taskEXIT_CRITICAL();
}
