#include <stdbool.h>
#include <stdint.h>

#include "main.h"
#include "MotorController.h"
#include "FreeRTOS.h"
#include "task.h"

extern TIM_HandleTypeDef htim2;
extern void Error_Handler(void);

/* -----------------------------------------------------------------------
 * State machine cho mỗi motor
 * ----------------------------------------------------------------------- */
typedef enum
{
    MOTOR_STATE_IDLE,      /* Đứng yên, PWM = 0                          */
    MOTOR_STATE_KICKSTART, /* Đang kích xung MAX để vượt ma sát tĩnh     */
    MOTOR_STATE_RUNNING,   /* Đang chạy ổn định ở target_speed           */
} MotorState_t;

typedef struct
{
    /* Trạng thái hiện tại */
    MotorState_t state;

    /* Chiều quay hiện tại đang áp lên H-bridge */
    Direction_t current_direction;

    /* Tốc độ đích người dùng yêu cầu (0 = dừng) */
    uint16_t target_speed;

    /* Chiều quay đích người dùng yêu cầu */
    Direction_t target_direction;

    /* Thời điểm (HAL_GetTick) khi bắt đầu kickstart */
    uint32_t kickstart_start_ms;

    /* Cờ: có yêu cầu mới từ bên ngoài chưa xử lý */
    volatile bool pending_request;
} MotorCtrl_t;

static MotorCtrl_t g_motors[2]; /* index 0 = MOTOR_1, index 1 = MOTOR_2 */
static TaskHandle_t motorControllerTaskHandler = NULL;

/* -----------------------------------------------------------------------
 * Helpers GPIO / PWM theo từng motor
 * ----------------------------------------------------------------------- */
static void Motor_ApplyDirection(MotorID_t id, Direction_t dir)
{
    if (id == MOTOR_1)
    {
        if (dir == FORWARD)
        {
            HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_RESET);
        }
        else
        {
            HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_SET);
        }
    }
    else /* MOTOR_2 */
    {
        if (dir == FORWARD)
        {
            HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, GPIO_PIN_RESET);
        }
        else
        {
            HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, GPIO_PIN_SET);
        }
    }
}

static void Motor_Coast(MotorID_t id)
{
    /* Tắt cả IN → motor quay trớn (coast stop) */
    if (id == MOTOR_1)
    {
        HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_RESET);
    }
    else
    {
        HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, GPIO_PIN_RESET);
    }
}

static void Motor_SetPWM(MotorID_t id, uint16_t pwm_value)
{
    uint32_t channel = (id == MOTOR_1) ? TIM_CHANNEL_1 : TIM_CHANNEL_2;
    __HAL_TIM_SET_COMPARE(&htim2, channel, pwm_value);
}

/* -----------------------------------------------------------------------
 * State machine tick — gọi mỗi vòng lặp task
 * ----------------------------------------------------------------------- */
static void Motor_Tick(MotorID_t id)
{
    MotorCtrl_t *m = &g_motors[id];
    uint32_t now = HAL_GetTick();

    switch (m->state)
    {
    /* ── IDLE ─────────────────────────────────────────────────────────── */
    case MOTOR_STATE_IDLE:
        if (m->pending_request && m->target_speed > 0)
        {
            m->pending_request = false;

            /* Áp chiều quay trước khi kích xung */
            m->current_direction = m->target_direction;
            Motor_ApplyDirection(id, m->current_direction);

            /* Kích xung MAX để vượt ma sát tĩnh */
            Motor_SetPWM(id, MAX_MOTOR_SPEED);
            m->kickstart_start_ms = now;
            m->state = MOTOR_STATE_KICKSTART;
        }
        break;

    /* ── KICKSTART ────────────────────────────────────────────────────── */
    case MOTOR_STATE_KICKSTART:
        if ((now - m->kickstart_start_ms) >= KICKSTART_DURATION_MS)
        {
            /* Hết thời gian kickstart → hạ về tốc độ đích */
            if (m->target_speed == 0)
            {
                /* Người dùng huỷ trong lúc kickstart */
                Motor_SetPWM(id, 0);
                Motor_Coast(id);
                m->state = MOTOR_STATE_IDLE;
            }
            else
            {
                Motor_SetPWM(id, m->target_speed);
                m->state = MOTOR_STATE_RUNNING;
            }
            m->pending_request = false;
        }
        /* Nếu có yêu cầu đổi chiều trong khi đang kickstart, cứ chờ
         * hết kickstart rồi mới xử lý ở lần tick tiếp theo           */
        break;

    /* ── RUNNING ──────────────────────────────────────────────────────── */
    case MOTOR_STATE_RUNNING:
        if (!m->pending_request)
        {
            break; /* Không có gì thay đổi */
        }
        m->pending_request = false;

        if (m->target_speed == 0)
        {
            /* Lệnh dừng */
            Motor_SetPWM(id, 0);
            Motor_Coast(id);
            m->state = MOTOR_STATE_IDLE;
        }
        else if (m->target_direction != m->current_direction)
        {
            /* Đổi chiều: phải kickstart lại để thắng quán tính ngược */
            Motor_SetPWM(id, 0); /* Tắt PWM trước khi đổi IN  */
            Motor_Coast(id);     /* Cho motor coast một nhịp   */

            m->current_direction = m->target_direction;
            Motor_ApplyDirection(id, m->current_direction);

            Motor_SetPWM(id, MAX_MOTOR_SPEED);
            m->kickstart_start_ms = now;
            m->state = MOTOR_STATE_KICKSTART;
        }
        else
        {
            /* Cùng chiều, chỉ thay đổi tốc độ → cập nhật trực tiếp */
            Motor_SetPWM(id, m->target_speed);
        }
        break;

    default:
        break;
    }
}

/* -----------------------------------------------------------------------
 * FreeRTOS Task
 * ----------------------------------------------------------------------- */
static void MotorControllerTask(void *args)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (1)
    {
        Motor_Tick(MOTOR_1);
        Motor_Tick(MOTOR_2);

        /* Tick mỗi 10 ms — đủ nhanh để kickstart timer chính xác */
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(10));
    }
}

/* -----------------------------------------------------------------------
 * Public API
 * ----------------------------------------------------------------------- */
void MotorController_Init(void)
{
    /* Khởi tạo state cho cả 2 motor */
    for (int i = 0; i < 2; i++)
    {
        g_motors[i].state = MOTOR_STATE_IDLE;
        g_motors[i].current_direction = FORWARD;
        g_motors[i].target_speed = 0;
        g_motors[i].target_direction = FORWARD;
        g_motors[i].pending_request = false;
    }

    /* Tắt H-bridge */
    HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, GPIO_PIN_RESET);

    /* Bắt đầu PWM */
    if (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2) != HAL_OK)
    {
        Error_Handler();
    }

    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);

    xTaskCreate(MotorControllerTask, "MotorCtrl",
                configMINIMAL_STACK_SIZE * 2, NULL, 5,
                &motorControllerTaskHandler);
}

void MotorController_SetMotorSpeed(MotorID_t motor_id, uint16_t speed, Direction_t direction)
{
    if (motor_id >= 2)
        return;

    if (speed > MAX_MOTOR_SPEED)
    {
        speed = MAX_MOTOR_SPEED;
    }
    else if (speed > 0 && speed < MOTOR_DEAD_ZONE)
    {
        /* Vùng chết: motor không thể quay ở PWM này.
         * Clamp lên ngưỡng tối thiểu thay vì để motor đứng yên
         * trong khi task vẫn nghĩ motor đang chạy.              */
        speed = MOTOR_DEAD_ZONE;
    }

    MotorCtrl_t *m = &g_motors[motor_id];
    m->target_speed = speed;
    m->target_direction = direction;
    m->pending_request = true;
    /* Task sẽ nhận request ở lần tick tiếp theo (tối đa 10 ms) */
}

void MotorController_StopAllMotors(void)
{
    MotorController_SetMotorSpeed(MOTOR_1, 0, g_motors[MOTOR_1].current_direction);
    MotorController_SetMotorSpeed(MOTOR_2, 0, g_motors[MOTOR_2].current_direction);
}
