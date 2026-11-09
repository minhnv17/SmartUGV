#!/usr/bin/env python3
"""
Test script điều khiển xe SmartUGV qua MAVLink (MANUAL_CONTROL).
Yêu cầu cài đặt:
    pip install pymavlink pyserial
"""

import time
import sys
from pymavlink import mavutil

# Cấu hình cổng COM và Baudrate kết nối với STM32
SERIAL_PORT = "COM16"  # Thay đổi thành cổng COM tương ứng trên máy tính của bạn
BAUD_RATE = 115200


def main():
    print(f"Đang kết nối tới SmartUGV qua {SERIAL_PORT} @ {BAUD_RATE} baud...")
    try:
        master = mavutil.mavlink_connection(SERIAL_PORT, baud=BAUD_RATE, source_system=1, source_component=191)
    except Exception as e:
        print(f"Lỗi kết nối cổng Serial: {e}")
        sys.exit(1)

    print("Đang chờ Heartbeat từ STM32...")
    msg = master.recv_match(type='HEARTBEAT', blocking=True, timeout=5)
    if msg:
        print(
            f"-> Đã nhận Heartbeat! Type: {msg.type}, Autopilot: {msg.autopilot}")
    else:
        print("-> Cảnh báo: Chưa nhận được Heartbeat (kiểm tra lại chân TX/RX hoặc baudrate). Vẫn tiếp tục test...")

    def send_drive(throttle, steer, duration_sec):
        """
        throttle: [-1000, 1000] (Dương = Tiến, Âm = Lùi)
        steer:    [-1000, 1000] (Dương = Rẽ phải, Âm = Rẽ trái)
        duration_sec: Thời gian giữ lệnh (giây)
        """
        # MAVLink MANUAL_CONTROL:
        # x: forward/backward
        # y: lateral / roll
        # z: thrust
        # r: yaw / steer
        start_time = time.time()
        print(
            f"Gửi lệnh: Throttle={throttle}, Steer={steer} trong {duration_sec}s...")
        while time.time() - start_time < duration_sec:
            master.mav.manual_control_send(
                master.target_system if master.target_system != 0 else 1,
                int(throttle),  # x
                0,              # y
                0,              # z
                int(steer),     # r
                0               # buttons
            )
            # Gửi chu kỳ 20Hz (mỗi 50ms) để không bị failsafe watchdog ngắt (>500ms)
            time.sleep(0.05)

    def stop():
        print("Dừng xe...")
        for _ in range(5):
            master.mav.manual_control_send(1, 0, 0, 0, 0, 0)
            time.sleep(0.02)

    def arm():
        print("Gửi lệnh ARM xe...")
        master.mav.command_long_send(
            1, 1,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1, 0, 0, 0, 0, 0, 0
        )
        time.sleep(0.5)

    def disarm():
        print("Gửi lệnh DISARM xe...")
        master.mav.command_long_send(
            1, 1,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            0, 0, 0, 0, 0, 0, 0
        )
        time.sleep(0.5)

    try:
        print("\n--- BẮT ĐẦU TEST ĐIỀU KHIỂN QUA MAVLINK ---")
        arm()

        # 1. Tiến thẳng tốc độ 800 trong 2 giây
        print("\n1. Tiến thẳng (Speed 800)")
        send_drive(throttle=800, steer=0, duration_sec=2.0)
        stop()
        time.sleep(1.0)

        # 2. Lùi thẳng tốc độ 800 trong 2 giây
        print("\n2. Lùi thẳng (Speed 800)")
        send_drive(throttle=-800, steer=0, duration_sec=2.0)
        stop()
        time.sleep(1.0)

        # 3. Rẽ trái tại chỗ trong 1.5 giây
        print("\n3. Rẽ trái tại chỗ")
        send_drive(throttle=0, steer=-800, duration_sec=1.5)
        stop()
        time.sleep(1.0)

        # 4. Rẽ phải tại chỗ trong 1.5 giây
        print("\n4. Rẽ phải tại chỗ")
        send_drive(throttle=0, steer=800, duration_sec=1.5)
        stop()
        time.sleep(1.0)

        # 5. Vừa tiến vừa rẽ phải nhẹ
        print("\n5. Vừa tiến vừa rẽ phải")
        send_drive(throttle=800, steer=300, duration_sec=2.0)
        stop()

        disarm()
        print("\n--- TEST HOÀN TẤT ---")

    except KeyboardInterrupt:
        print("\nNgười dùng ngắt chương trình! Dừng xe và DISARM khẩn cấp...")
        stop()
        disarm()


if __name__ == '__main__':
    main()
