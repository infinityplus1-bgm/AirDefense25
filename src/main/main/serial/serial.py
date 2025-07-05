import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16, UInt8, Int32, String
import serial
import struct # For packing data into bytes
import logging
from main.utils.logging_config import setup_logging
from main.config import SERIAL_PORT, BAUD_RATE
from main import config as cfg

logger = logging.getLogger(__name__)

class SerialNode(Node):
    def __init__(self):
        super().__init__(cfg.NODE_SERIAL)
        # TODO: take as paramters using argparse
        self.declare_parameter('serial_port', SERIAL_PORT)  
        self.declare_parameter('baud_rate', BAUD_RATE) 

        self.serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        self.baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value

        self.system_enabled = False

        
        logger.info(f"Attempting to open serial port: {self.serial_port} at {self.baud_rate} baud.")
        try:
            self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
            logger.info("Serial port opened successfully.")
        except serial.SerialException as e:
            logger.error(f"Error opening serial port: {e}")
            rclpy.shutdown()
            return

        # Create subscribers for laser PWM and motor steps
        self.laser_pwm_subscription = self.create_subscription(
            UInt8,
            cfg.TOPIC_LASER_COMMAND,
            self.laser_pwm_callback,
            10
        )
        self.motor_pan_subscription = self.create_subscription(
            Int16,
            cfg.TOPIC_MOTOR_PAN,
            self.motor_pan_callback,
            10
        )
        self.motor_tilt_subscription = self.create_subscription(
            Int16,
            cfg.TOPIC_MOTOR_TILT,
            self.motor_tilt_callback,
            10
        )

        self.status_subscriber = self.create_subscription(
            Int32,
            cfg.TOPIC_SYSTEM_STATUS,
            self.system_status_callback,
            10
        )

        self.laser_pwm = 0
        self.motor_pan = 0
        self.motor_tilt = 0

        # Timer to periodically send the combined message
        # TODO: take as cli parameter
        self.send_timer = self.create_timer(1, self.send_serial_message)
        self.health_publisher = self.create_publisher(String, cfg.TOPIC_HEALTH_SERIAL_NODE, 10)
        self.health_timer = self.create_timer(1, self.publish_health_status)

        logger.info('Serial Node initialized.')

    def publish_health_status(self):
        msg = String()
        msg.data = 'healthy'
        self.health_publisher.publish(msg)

    def system_status_callback(self, msg):
        self.system_enabled = bool(msg.data)
        if self.system_enabled:
            logger.info('System enabled. Serial node is active.')
        else:
            logger.info('System disabled. Serial node is inactive.')

    def laser_pwm_callback(self, msg):
        if not self.system_enabled:
            return
        self.laser_pwm = msg.data
        # make sure it doesn't exceep 255 maximum for one byte
        if not (0 <= self.laser_pwm <= 255):
            logger.warning(f"Laser PWM value out of range (0-255): {self.laser_pwm}. Clamping.")
            self.laser_pwm = max(0, min(255, self.laser_pwm))

    def motor_pan_callback(self, msg):
        if not self.system_enabled:
            return
        self.motor_pan = msg.data

    def motor_tilt_callback(self, msg):
        if not self.system_enabled:
            return
        self.motor_tilt = msg.data

    def send_serial_message(self):
        if not self.system_enabled:
            return
        # Pack the data into 5 bytes:
        # Byte 0: Laser PWM (unsigned 8-bit)
        # Bytes 1-2: Motor A steps (signed 16-bit, little-endian)
        # Bytes 3-4: Motor B steps (signed 16-bit, little-endian)

        # '<bhh' format string:
        # '<' : little-endian
        # 'B' : unsigned char (1 byte)
        # 'h' : signed short (2 bytes)

        try:
            # Ensure laser_pwm is within 0-255 range
            packed_data = struct.pack('<Bhh', self.laser_pwm, self.motor_pan, self.motor_tilt)
            self.ser.write(packed_data)
            logger.info(f"Sent: Laser PWM: {self.laser_pwm}, Motor A Steps: {self.motor_pan}, Motor B Steps: {self.motor_tilt} (Bytes: {packed_data.hex()})")
            # TODO: after sending reset all values
        except serial.SerialException as e:
            logger.error(f"Error sending data over serial: {e}")
        except struct.error as e:
            logger.error(f"Error packing data: {e}. Check data types and ranges.")

    def destroy_node(self):
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()
            logger.info("Serial port closed.")
        super().destroy_node()

def main(args=None):
    setup_logging()
    rclpy.init(args=args)
    # initialize the node
    node = SerialNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # destroy the node by calling the method to ensure closing the serial as well
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
