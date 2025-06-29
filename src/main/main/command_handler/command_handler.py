import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String, Int32MultiArray

class CommandHandler(Node):
    def __init__(self):
        super().__init__('command_handler')

        self.current_mode = "idle" # will take the value from the topic system/mode
        self.mode_sub = self.create_subscription(String, '/system/mode', self.mode_callback, 10)
        self.command_sub = self.create_subscription(Float32MultiArray, '/system/commands', self.command_callback, 10)
        self.imageproc_sub = self.create_subscription(Float32MultiArray, '/imageproc/results', self.imageproc_callback, 10)
        self.serial_pub = self.create_publisher(Int32MultiArray, '/serial/commands', 10)

        #  COMMAND FUNCTION table
        self.command_table = {
            0: self.move_right,
            1: self.move_left,
            2: self.shoot,
            3: self.stop,
        }

        self.get_logger().info('Command Handler node initialized.')


    def mode_callback(self, msg):
        new_mode = msg.data
        if new_mode != self.current_mode:
            self.get_logger().info(f"Mode changed: {self.current_mode} -> {new_mode}")
            self.current_mode = new_mode

    def command_callback(self, msg):
        if len(msg.data) < 2:
            self.get_logger().warn("Received invalid command message.")
            return

        command_id = int(msg.data[0])
        value = msg.data[1]

        if command_id in self.command_table:
            self.get_logger().info(f"Executing command ID {command_id} with value {value}")
            self.command_table[command_id](value)
        else:
            self.get_logger().warn(f"Unknown command ID: {command_id}")

    def imageproc_callback(self, msg):
        self.get_logger().info(f"Received image processing result: {msg.data}")
        # Use this to implement tracking logic in "auto" mode later

    # COMMAND FUNCTIONS

    def move_right(self, steps):
        self.get_logger().info(f"[COMMAND] Move right by {steps} steps")
        self.publish_serial_command([0, int(steps)])

    def move_left(self, steps):
        self.get_logger().info(f"[COMMAND] Move left by {steps} steps")
        self.publish_serial_command([1, int(steps)])

    def shoot(self, power):
        self.get_logger().info(f"[COMMAND] Shoot with power {power}")
        self.publish_serial_command([2, int(power)])

    def stop(self, duration):
        self.get_logger().info(f"[COMMAND] Stop for {duration} seconds")
        self.publish_serial_command([3, int(duration)])

    # PUBLISH SERIAL COMMANDS

    def publish_serial_command(self, command_list):
        msg = Int32MultiArray()
        msg.data = command_list
        self.serial_pub.publish(msg)
        self.get_logger().info(f"Published to /serial/commands: {command_list}")

# === MAIN ===

def main(args=None):
    rclpy.init(args=args)
    node = CommandHandler()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
