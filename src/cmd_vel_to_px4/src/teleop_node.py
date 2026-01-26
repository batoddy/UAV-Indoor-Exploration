#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import termios
import tty

class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_node')
        
        self.declare_parameter('v_max', 2.0)
        self.v_max = self.get_parameter('v_max').value
        
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.settings = termios.tcgetattr(sys.stdin)
        
        self.get_logger().info(f'Teleop başlatıldı. Hız: {self.v_max} m/s')
        self.get_logger().info('w: ileri, s: geri, a: sola dön, d: sağa dön')
        self.get_logger().info('Shift: yukarı, Ctrl: aşağı, q: çıkış')
        
    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key
    
    def run(self):
        try:
            while rclpy.ok():
                key = self.get_key()
                
                twist = Twist()
                
                if key == 'w':
                    twist.linear.x = self.v_max
                elif key == 's':
                    twist.linear.x = -self.v_max
                elif key == 'a':
                    twist.angular.z = 1.5
                elif key == 'd':
                    twist.angular.z = -1.5
                elif key == 'W':  # Shift+w
                    twist.linear.z = self.v_max
                elif key == '\x17':  # Ctrl+w
                    twist.linear.z = -self.v_max
                elif key == 'q' or key == '\x03':  # q veya Ctrl+C
                    break
                
                self.cmd_vel_pub.publish(twist)
                
        except Exception as e:
            self.get_logger().error(f'Hata: {e}')
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()