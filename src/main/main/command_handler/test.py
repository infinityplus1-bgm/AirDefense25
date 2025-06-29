import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import time

class TestCommandPublisher(Node):
    def __init__(self):
        super().__init__('test_command_publisher')

        self.publisher_ = self.create_publisher(Float32MultiArray, '/system/command', 10)
        self.timer = self.create_timer(2.0, self.timer_callback)  # Every 2 seconds

        # List of test commands: [command_id, value]
        self.commands = [
            [0, 2000],  # move right
            [1, 1500],  # move left
            [2, 1],     # shoot
            [3, 5],     # stop
        ]
        self.index = 0

    def timer_callback(self):
        if self.index < len(self.commands):
            cmd = self.commands[self.index]
            msg = Float32MultiArray()
            msg.data = cmd
            self.publisher_.publish(msg)
            self.get_logger().info(f'Published command: ID={cmd[0]}, Value={cmd[1]}')
            self.index += 1
        else:
            self.get_logger().info('All test commands sent. Shutting down...')
            self.destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = TestCommandPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
