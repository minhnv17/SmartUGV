#ifndef COMMAND_PARSER_H
#define COMMAND_PARSER_H

#include <stdint.h>

/**
 * @brief Giao thức ASCII đơn giản để test điều khiển motor qua UART.
 *
 * Định dạng: <lệnh>[tốc_độ]\n
 *
 *   w<speed>\n  — Tiến  (Forward),  cả 2 bánh, tốc độ = speed
 *   b<speed>\n  — Lùi   (Backward), cả 2 bánh, tốc độ = speed
 *   a<speed>\n  — Rẽ trái (bánh trái lùi, bánh phải tiến)
 *   d<speed>\n  — Rẽ phải (bánh trái tiến, bánh phải lùi)
 *   s\n         — Dừng (stop all)
 *
 * Ví dụ gửi từ terminal (minicom / PuTTY / Python):
 *   w800\n   → tiến tốc độ 800
 *   s\n      → dừng
 *
 * speed nằm ngoài [MOTOR_DEAD_ZONE, MAX_MOTOR_SPEED] sẽ được clamp tự động.
 */

/**
 * @brief Feed từng byte UART vào parser.
 *        Gọi hàm này trong ISR / DMA callback cho mỗi byte nhận được.
 *        Khi nhận đủ một dòng (\n hoặc \r\n), parser tự động thực thi lệnh.
 */
void CommandParser_FeedByte(uint8_t byte);

#endif /* COMMAND_PARSER_H */
