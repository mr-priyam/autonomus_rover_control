import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import time
import math

class SerialCmdVel(Node):
    def __init__(self):
        super().__init__('serial_cmd_vel')

        self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.1)
        time.sleep(2)

        self.sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_callback,
            10
        )

        # Rover geometry (meters)
        self.L = 1.0   # wheelbase
        self.W = 1.0   # track width
        # self.r = 2.0  # wheel radius (1 m)

        self.EPS = 1e-6
    
    def clamp(self, val, min_val, max_val):
        return max(min(val, max_val), min_val)


    def map_range(self, x, in_min, in_max, out_min, out_max):
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


    def cmd_callback(self, msg):
        vx = msg.linear.x
        wz = msg.angular.z

        L = self.L
        W = self.W

        # ------------------------------------------------------------
        # Case 1: Straight motion
        # ------------------------------------------------------------
        # ------------------------------------------------------------
        # Motion cases
        # ------------------------------------------------------------
        if abs(vx) < self.EPS and abs(wz) > self.EPS:
            # In-place rotation → max steering
            steer = 45.0  # degrees (tune this)
            d_fl =  math.radians(steer)
            d_fr = -math.radians(steer)
            d_rl = -math.radians(steer)
            d_rr =  math.radians(steer)

            v_fl = v_fr = v_rl = v_rr = abs(wz)

        elif abs(wz) < self.EPS:
            # Straight motion
            v_fl = v_fr = v_rl = v_rr = abs(vx)
            d_fl = d_fr = d_rl = d_rr = 0.0

        else:
            # Ackermann steering
            d_fl = math.atan((L * wz) / (2 * vx - wz * W))
            d_fr = math.atan((L * wz) / (2 * vx + wz * W))
            d_rl = -d_fl
            d_rr = -d_fr

        v_left  = math.sqrt((vx - wz * W / 2)**2 + (wz * L / 2)**2)
        v_right = math.sqrt((vx + wz * W / 2)**2 + (wz * L / 2)**2)

        v_fl = v_rl = v_left*2
        v_fr = v_rr = v_right*2


        # ------------------------------------------------------------
        # Convert steering angles to degrees (servo side)
        # ------------------------------------------------------------
        # Convert to degrees
        d_fl_deg = math.degrees(d_fl)
        d_fr_deg = math.degrees(d_fr)
        d_rl_deg = math.degrees(d_rl)
        d_rr_deg = math.degrees(d_rr)

        # ------------------------------------------------------------
        # 1. Constrain angles to [-90, +90]
        # ------------------------------------------------------------
        d_fl_deg = self.clamp(d_fl_deg, -90.0, 90.0)
        d_fr_deg = self.clamp(d_fr_deg, -90.0, 90.0)
        d_rl_deg = self.clamp(d_rl_deg, -90.0, 90.0)
        d_rr_deg = self.clamp(d_rr_deg, -90.0, 90.0)

        # ------------------------------------------------------------
        # 2. Map [-90, +90] → [0, 180] for servo
        # ------------------------------------------------------------
        d_fl_servo = self.map_range(d_fl_deg, -90.0, 90.0, 0.0, 180.0)
        d_fr_servo = self.map_range(d_fr_deg, -90.0, 90.0, 0.0, 180.0)
        d_rl_servo = self.map_range(d_rl_deg, -90.0, 90.0, 0.0, 180.0)
        d_rr_servo = self.map_range(d_rr_deg, -90.0, 90.0, 0.0, 180.0)

        # ------------------------------------------------------------
        # Serial packet (exact format you use)
        # ------------------------------------------------------------
        cmd = (
            f"{v_fl:.1f}:{v_fr:.1f}:{d_fl_servo:.1f}:{d_fr_servo:.1f}:{d_rl_servo:.1f}:{d_rr_servo:.1f} \n"
        )

        self.ser.write(cmd.encode())

def main(args=None):
    rclpy.init(args=args)
    node = SerialCmdVel()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
