#include <string.h>
#include <stdlib.h>
#include <ctype.h>

#include "CommandParser.h"
#include "MotorController.h"

#define CMD_BUF_SIZE 16u

static char     s_buf[CMD_BUF_SIZE];
static uint8_t  s_len = 0;

/* -----------------------------------------------------------------------
 * Thực thi lệnh sau khi nhận đủ một dòng
 * ----------------------------------------------------------------------- */
static void CommandParser_Execute(void)
{
    if (s_len == 0) return;

    char    cmd   = s_buf[0];          /* Ký tự lệnh đầu tiên              */
    uint16_t speed = 0;

    /* Parse số tốc độ phía sau ký tự lệnh (nếu có) */
    if (s_len > 1)
    {
        speed = (uint16_t)atoi(&s_buf[1]);
    }

    switch (cmd)
    {
    /* ── Tiến thẳng ─────────────────────────────────────────────────── */
    case 'w':
    case 'W':
        MotorController_SetMotorSpeed(MOTOR_1, speed, FORWARD);
        MotorController_SetMotorSpeed(MOTOR_2, speed, FORWARD);
        break;

    /* ── Lùi thẳng ──────────────────────────────────────────────────── */
    case 'b':
    case 'B':
        MotorController_SetMotorSpeed(MOTOR_1, speed, REVERSE);
        MotorController_SetMotorSpeed(MOTOR_2, speed, REVERSE);
        break;

    /* ── Rẽ trái: bánh trái lùi, bánh phải tiến ─────────────────────── */
    case 'a':
    case 'A':
        MotorController_SetMotorSpeed(MOTOR_1, speed, REVERSE);
        MotorController_SetMotorSpeed(MOTOR_2, speed, FORWARD);
        break;

    /* ── Rẽ phải: bánh trái tiến, bánh phải lùi ─────────────────────── */
    case 'd':
    case 'D':
        MotorController_SetMotorSpeed(MOTOR_1, speed, FORWARD);
        MotorController_SetMotorSpeed(MOTOR_2, speed, REVERSE);
        break;

    /* ── Dừng ────────────────────────────────────────────────────────── */
    case 's':
    case 'S':
        MotorController_StopAllMotors();
        break;

    default:
        /* Lệnh không hợp lệ — bỏ qua */
        break;
    }
}

/* -----------------------------------------------------------------------
 * Public API — gọi cho mỗi byte nhận được từ UART
 * ----------------------------------------------------------------------- */
void CommandParser_FeedByte(uint8_t byte)
{
    /* ── Các ký tự kết thúc dòng ──────────────────────────────────────────
     * Chấp nhận:
     *   0x0A (\n thật)  — terminal chuẩn, Python serial, v.v.
     *   0x0D (\r thật)  — một số tool chỉ gửi CR
     *
     * Hercules text-mode gửi ký tự '\' (0x5C) + 'n' (0x6E) thay vì 0x0A.
     * Trường hợp đó được bắt bên dưới khi phát hiện 'n' đứng sau '\'.    */

    if (byte == '\n' || byte == '\r')
    {
        /* Bỏ chuỗi chỉ có mỗi '\' còn dư (từ literal \n của Hercules) */
        if (s_len == 1 && s_buf[0] == '\\')
        {
            s_len = 0;
            return;
        }
        /* Kết thúc dòng bình thường */
        s_buf[s_len] = '\0';
        CommandParser_Execute();
        s_len = 0;
        return;
    }

    /* ── Phát hiện literal "\n" từ Hercules text-mode ────────────────────
     * Hercules gửi 0x5C ('\\') rồi 0x6E ('n').
     * Khi thấy 'n' đứng ngay sau '\\', coi như nhận được terminator.     */
    if (byte == 'n' && s_len >= 1 && s_buf[s_len - 1] == '\\')
    {
        s_len--;                  /* Xoá ký tự '\\' khỏi buffer           */
        s_buf[s_len] = '\0';
        CommandParser_Execute();
        s_len = 0;
        return;
    }

    /* ── Tích luỹ ký tự vào buffer ───────────────────────────────────────*/
    if (s_len < CMD_BUF_SIZE - 1)
    {
        s_buf[s_len++] = (char)byte;
    }
    else
    {
        /* Buffer đầy mà chưa thấy terminator → lệnh rác, reset */
        s_len = 0;
    }
}
