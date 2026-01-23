#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import serial
import time

class JoyToSerial(Node):

    def __init__(self):
        super().__init__('joy_to_serial')

        # ---------- PARAMETERS ----------
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)

        port = self.get_parameter('serial_port').value
        baud = self.get_parameter('baudrate').value

        # ---------- SERIAL ----------
        try:
            self.ser = serial.Serial(port, baud, timeout=0.1)
            # Short sleep to allow some microcontrollers (like Arduino) to reset
            time.sleep(2)  
            self.get_logger().info(f'Serial connected: {port} @ {baud}')
        except Exception as e:
            self.get_logger().error(f'Serial connection failed: {e}')
            self.ser = None

        # ---------- BUTTON ORDER ----------
        # Required order: 4, 0, 3, 1, 8, 9
        self.button_order = [4, 0, 3, 1, 8, 9]

        # ---------- SUBSCRIBER ----------
        self.sub = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10
        )

    def joy_callback(self, msg: Joy):
        # ---------- SAFETY CHECK ----------
        # Ensure the message actually contains the buttons/axes we expect
        if len(msg.axes) < 4 or len(msg.buttons) < max(self.button_order) + 1:
            return

        # ---------- AXES ----------
        axis_7 = msg.axes[7]
        axis_3 = msg.axes[3]

        # ---------- BUTTONS (MOMENTARY/LEVEL MODE) ----------
        # This list comprehension will result in 1 if held, 0 if released.
        button_values = [msg.buttons[i] for i in self.button_order]

        # ---------- SERIAL FORMAT ----------
        # Example output: "A3 0.123 A7 -0.456 B 1 0 0 1 0 0\n"
        btn_str = ' '.join(map(str, button_values))
        data = f"A3 {axis_3:.3f} A7 {axis_7:.3f} B {btn_str}\n"

        if self.ser and self.ser.is_open:
            try:
                self.ser.write(data.encode())
                # Log only when a button is pressed to avoid flooding the terminal
                self.get_logger().info(f"Sending: {data.strip()}")

            except Exception as e:
                self.get_logger().error(f"Write failed: {e}")

    def destroy_node(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
        super().destroy_node()

def main():
    rclpy.init()
    node = JoyToSerial()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()