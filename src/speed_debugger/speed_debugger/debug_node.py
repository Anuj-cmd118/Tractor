import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class SpeedDebugger(Node):
    def __init__(self):
        super().__init__('speed_debugger')
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)
        self.vel_sub = self.create_subscription(Twist, '/vehicle/velocity', self.vel_callback, 10)

        self.cmd_speed = 0.0
        self.current_speed = 0.0

    def cmd_callback(self, msg):
        self.cmd_speed = msg.linear.x
        self.print_error()

    def vel_callback(self, msg):
        self.current_speed = msg.linear.x
        self.print_error()

    def print_error(self):
        error = self.cmd_speed - self.current_speed
        self.get_logger().info(
            f"Set Speed: {self.cmd_speed:.2f}, Current Speed: {self.current_speed:.2f}, Error: {error:.2f}"
        )

def main(args=None):
    rclpy.init(args=args)
    node = SpeedDebugger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
