#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include <stdint.h>

typedef enum
{
    MOTOR_1 = 0,
    MOTOR_2 = 1,
} MotorID_t;

typedef enum
{
    FORWARD = 0,
    REVERSE = 1,
} Direction_t;

#define MAX_MOTOR_SPEED 1000u /* PWM compare value tương ứng 100% duty  */
#define MIN_MOTOR_SPEED 0u    /* 0 = dừng hẳn                           */

/**
 * Vùng chết (dead zone): PWM dưới ngưỡng này không đủ điện áp để
 * thắng ma sát trong motor — motor đứng im dù có tín hiệu.
 *
 * Giá trị hợp lệ để motor thực sự chạy: [MOTOR_DEAD_ZONE, MAX_MOTOR_SPEED]
 * Mọi speed != 0 sẽ được clamp vào vùng này tự động trong SetMotorSpeed().
 */
#define MOTOR_DEAD_ZONE 750u /* PWM tối thiểu để motor quay được       */

#define KICKSTART_DURATION_MS 80u /* Thời gian kích xung max khi khởi động  */

void MotorController_Init(void);

/**
 * @brief Đặt tốc độ + chiều quay cho một motor.
 *
 * Hàm này KHÔNG blocking. Nếu motor đang đứng yên hoặc đổi chiều,
 * MotorControllerTask sẽ tự động kích xung MAX trước rồi mới
 * giảm về @p speed sau KICKSTART_DURATION_MS ms.
 *
 * @param motor_id  MOTOR_1 hoặc MOTOR_2
 * @param speed     0 = dừng, 1..MAX_MOTOR_SPEED = tốc độ mong muốn
 * @param direction FORWARD hoặc REVERSE
 */
void MotorController_SetMotorSpeed(MotorID_t motor_id, uint16_t speed, Direction_t direction);

/**
 * @brief Dừng khẩn cấp (coast stop) tất cả motor ngay lập tức.
 */
void MotorController_StopAllMotors(void);

#endif /* MOTOR_CONTROLLER_H */