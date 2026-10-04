#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include <stdint.h>

typedef enum
{
    MOTOR_1 = 1,
    MOTOR_2 = 2,
} MotorID_t;

typedef enum
{
    FORWARD = 0,
    REVERSE = 1,
} Direction_t;

#define MAX_MOTOR_SPEED 1000 // Giá trị PWM tối đa (tương ứng với 100% duty cycle)

void MotorController_Init(void);
void MotorController_SetMotorSpeed(MotorID_t motor_id, uint16_t speed, Direction_t direction);
void MotorController_StopAllMotors(void);
#endif // MOTOR_CONTROLLER_H