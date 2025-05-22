import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16
from std_msgs.msg import UInt8

class SerialTestNode(Node):
    def __init__(self):
        super().__init__('serial_test')

        # Create publishers for the topics that the serial node subscribes to
        self.laser_pwm_publisher = self.create_publisher(UInt8, 'laser/command', 10)
        self.motor_pan_publisher = self.create_publisher(Int16, 'motor/pan', 10)
        self.motor_tilt_publisher = self.create_publisher(Int16, 'motor/tilt', 10)

        self.get_logger().info('User Input Publisher Node initialized. Enter values in the terminal.')
        self.get_logger().info('Press Ctrl+C to exit.')

        # Start the input loop in a separate thread or use a timer if non-blocking input is needed.
        # For simple CLI interaction, a blocking loop is fine.
        self.timer = self.create_timer(0.5, self.prompt_for_input) # Use a timer to keep node spinning

    def prompt_for_input(self):
        """
        Prompts the user for input and publishes the values.
        This method is called by a timer to allow the ROS 2 node to spin.
        """
        try:
            # Get Laser PWM input
            laser_pwm_str = input("Enter Laser PWM (0-255): ")
            laser_pwm = int(laser_pwm_str)
            if not (0 <= laser_pwm <= 255):
                self.get_logger().warn("Laser PWM out of valid range (0-255). Clamping.")
                laser_pwm = max(0, min(255, laser_pwm))
            
            # Get Motor Pan Steps input
            motor_pan_str = input("Enter Motor Pan Steps (signed 16-bit, e.g., -32768 to 32767): ")
            motor_pan = int(motor_pan_str)

            # Get Motor Tilt Steps input
            motor_tilt_str = input("Enter Motor Tilt Steps (signed 16-bit, e.g., -32768 to 32767): ")
            motor_tilt = int(motor_tilt_str)

            # Create messages
            laser_msg = UInt8()
            laser_msg.data = laser_pwm

            pan_msg = Int16()
            pan_msg.data = motor_pan

            tilt_msg = Int16()
            tilt_msg.data = motor_tilt

            # Publish messages
            self.laser_pwm_publisher.publish(laser_msg)
            self.motor_pan_publisher.publish(pan_msg)
            self.motor_tilt_publisher.publish(tilt_msg)

            self.get_logger().info(f"Published: Laser PWM={laser_pwm}, Pan Steps={motor_pan}, Tilt Steps={motor_tilt}")

        except ValueError:
            self.get_logger().error("Invalid input. Please enter integer values.")
        except KeyboardInterrupt:
            self.get_logger().info("Exiting user input prompt.")
            # This will be caught by the main loop's try-except as well, leading to shutdown.
            raise # Re-raise to break out of rclpy.spin

def main(args=None):
    rclpy.init(args=args)
    node = SerialTestNode()
    try:
        # Use rclpy.spin_once with a small timeout to allow the timer to trigger
        # and also handle potential KeyboardInterrupts more gracefully.
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()