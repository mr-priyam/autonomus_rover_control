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
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)

        port = self.get_parameter('serial_port').value
        baud = self.get_parameter('baudrate').value

        # ---------- SERIAL ----------
        try:
            self.ser = serial.Serial(port, baud, timeout=0.1)
            time.sleep(2)  # allow Arduino reset
            self.get_logger().info(f'Serial connected: {port} @ {baud}')
        except Exception as e:
            self.get_logger().error(f'Serial connection failed: {e}')
            self.ser = None

        # ---------- SUBSCRIBER ----------
        self.sub = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10
        )

    def joy_callback(self, msg: Joy):
        # Safety check
        if len(msg.axes) < 4:
            self.get_logger().warn('Joy message has insufficient axes')
            return

        axis_3 = msg.axes[3]
        axis_1 = msg.axes[1]

        # Format: A3 <val> A1 <val>\n
        data = f"A3 {axis_3:.3f} A1 {axis_1:.3f}\n"

        if self.ser and self.ser.is_open:
            self.ser.write(data.encode())
        else:
            self.get_logger().warn('Serial not available')

def main():
    rclpy.init()
    node = JoyToSerial()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
