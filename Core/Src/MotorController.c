#include "main.h"

#include "MotorController.h"

extern TIM_HandleTypeDef htim2; // Đảm bảo rằng htim2 được khai báo ở nơi khác, ví dụ trong main.c
extern void Error_Handler(void);

void MotorController_Init(void)
{
    /* Stop all motors */
    HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, GPIO_PIN_RESET);

    if (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1) != HAL_OK) // Bắt đầu PWM cho kênh 1
    {
        Error_Handler(); // Trả về lỗi
    }
    if (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2) != HAL_OK) // Bắt đầu PWM cho kênh 2
    {
        Error_Handler(); // Trả về lỗi
    }

    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
}

void MotorController_SetMotorSpeed(MotorID_t motor_id, uint16_t speed, Direction_t direction)
{
    if (speed > MAX_MOTOR_SPEED) // Giới hạn tốc độ tối đa
    {
        speed = MAX_MOTOR_SPEED;
    }

    switch (motor_id)
    {
    case MOTOR_1:
        if (direction == FORWARD)
        {
            HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_RESET);
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, speed); // Cập nhật giá trị PWM cho kênh 1
        }
        else if (direction == REVERSE)
        {
            HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_SET);
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, speed); // Cập nhật giá trị PWM cho kênh 1
        }

        break;

    case MOTOR_2:
        if (direction == FORWARD)
        {
            HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, GPIO_PIN_RESET);
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, speed); // Cập nhật giá trị PWM cho kênh 2
        }
        else if (direction == REVERSE)
        {
            HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, GPIO_PIN_SET);
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, speed); // Cập nhật giá trị PWM cho kênh 2
        }

        break;

    default:
        break;
    }
}

void MotorController_StopAllMotors(void)
{
    HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, GPIO_PIN_RESET);

    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0); // Dừng PWM cho kênh 1
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0); // Dừng PWM cho kênh 2
}