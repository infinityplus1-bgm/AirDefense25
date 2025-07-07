import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32, UInt8, Int16
from interfaces.msg import Command, Float32MultiArray2D
import logging
from main.utils.logging_config import setup_logging
from main.utils import commands as cmd
from main import config
import numpy as np
import serial
import struct


logger = logging.getLogger(__name__)

class CommandHandler(Node):
    def __init__(self):
        super().__init__(config.NODE_COMMAND_HANDLER)

        self.current_mode = config.MODE_MANUAL
        self.system_enabled = False
        self.latest_tracking_results = None

        # Serial communication setup
        self.declare_parameter('serial_port', config.SERIAL_PORT)
        self.declare_parameter('baud_rate', config.BAUD_RATE)
        self.serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        self.baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value
        
        try:
            self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
            logger.info(f"Serial port {self.serial_port} opened successfully at {self.baud_rate} baud.")
        except serial.SerialException as e:
            logger.error(f"Error opening serial port: {e}")
            rclpy.shutdown()
            return

        # Subscribers
        self.status_sub = self.create_subscription(Int32, config.TOPIC_SYSTEM_STATUS, self.status_callback, 10)
        self.mode_sub = self.create_subscription(String, config.TOPIC_SYSTEM_MODE, self.mode_callback, 10)
        self.ui_command_sub = self.create_subscription(Command, config.TOPIC_SYSTEM_COMMANDS, self.ui_command_callback, 10)
        self.tracking_sub = self.create_subscription(Float32MultiArray2D, config.TOPIC_RESULTS, self.tracking_results_callback, 10)
        
        # Health publisher
        self.health_publisher = self.create_publisher(String, config.TOPIC_HEALTH_COMMAND_HANDLER_NODE, 10)
        self.health_timer = self.create_timer(1, self.publish_health_status)
        
        logger.info('Command Handler node initialized')

    def publish_health_status(self):
        msg = String()
        msg.data = 'healthy'
        self.health_publisher.publish(msg)

    def status_callback(self, msg):
        self.system_enabled = bool(msg.data)
        logger.info(f"System status updated: {'Enabled' if self.system_enabled else 'Disabled'}")

    def mode_callback(self, msg):
        new_mode = msg.data
        if new_mode != self.current_mode:
            logger.info(f'System mode changed: {self.current_mode} -> {new_mode}')
            self.current_mode = new_mode

    def tracking_results_callback(self, msg : Float32MultiArray2D):
        # get the results back in shape
        self.latest_tracking_results = np.array(msg.data, dtype=np.float32).reshape(msg.rows, msg.cols)
        # self.latest_tracking_results = msg

        if self.current_mode == config.MODE_MANUAL:
            pass
        elif self.current_mode == config.MODE_PHASE_ONE:
            pass
        elif self.current_mode == config.MODE_PHASE_TWO:
            pass
        elif self.current_mode == config.MODE_PHASE_THREE:
            pass

    def ui_command_callback(self, msg : Command):

        if not self.system_enabled:
            logger.warning("System is disabled. Ignoring command.")
            return
        

        logger.info(f"received : {msg.id}")

        command_handlers = {
            cmd.CMD_MOVE_RIGHT: self.handle_movement,
            cmd.CMD_MOVE_LEFT: self.handle_movement,
            cmd.CMD_MOVE_UP: self.handle_movement,
            cmd.CMD_MOVE_DOWN: self.handle_movement,
            cmd.CMD_SHOOT: self.handle_shoot,
            cmd.CMD_SELECT_TARGET: self.handle_select_target,
            cmd.CMD_SET_NO_FIRE_ZONE: self.handle_set_no_fire_zone,
            cmd.CMD_LASER_POWER_CONTROL: self.handle_laser_power_control,
            cmd.CMD_PID_VALUES: self.handle_pid_values,
            cmd.CMD_LASER_LENS_DISTANCE: self.handle_laser_lens_distance,
            cmd.CMD_CLEAR_NO_FIRE_ZONE: self.handle_clear_no_fire_zone,
            cmd.CMD_RETURN_HOME: self.handle_return_home,
        }

        handler = command_handlers.get(msg.id)
        if handler:
            handler(msg)
        else:
            logger.warning(f"No handler for command ID: {msg.id}")

    def handle_movement(self, command : Command):
        if self.current_mode != config.MODE_MANUAL:
            logger.warning("Movement command received, but not in manual mode.")
            return
        logger.info(f"Handling movement: {cmd.COMMAND_DESCRIPTIONS[command.id]}")
        

    def handle_shoot(self, command):
        if self.current_mode not in (config.MODE_MANUAL, config.MODE_PHASE_THREE):
            logger.warning(f"Shoot command received, but not in an appropriate mode. Current mode: {self.current_mode}")
            return
        logger.info("Handling shoot command")

    def handle_select_target(self, command):
        if self.current_mode != config.MODE_PHASE_ONE:
            logger.warning(f"Select target command received, but not in phase 1. Current mode: {self.current_mode}")
            return
        logger.info(f"Handling select target: {command.values}")
        # Additional logic for target selection will be added here
        # self.serial_command_publisher.publish(command)

    def send_serial_message(self, laser_pwm, motor_pan, motor_tilt):
        if not self.system_enabled:
            return
        try:
            packed_data = struct.pack('<Bhh', laser_pwm, motor_pan, motor_tilt)
            self.ser.write(packed_data)
            logger.info(f"Sent: Laser PWM: {laser_pwm}, Motor Pan: {motor_pan}, Motor Tilt: {motor_tilt} (Bytes: {packed_data.hex()})")
        except serial.SerialException as e:
            logger.error(f"Error sending data over serial: {e}")
        except struct.error as e:
            logger.error(f"Error packing data: {e}. Check data types and ranges.")

    def handle_set_no_fire_zone(self, command):
        logger.info(f"Handling set no-fire zone: {command.values}")
        # Logic to handle no-fire zone would be implemented here
        # For now, we just log it.

    def handle_laser_power_control(self, command):
        logger.info(f"Handling laser power control: {command.values}")
        power = int(command.values[0])
        if not (0 <= power <= 255):
            logger.warning(f"Laser PWM value out of range (0-255): {power}. Clamping.")
            power = max(0, min(255, power))
        self.send_serial_message(power, 0, 0)

    def handle_pid_values(self, command):
        logger.info(f"Handling PID values: {command.values}")
        # PID values are not sent over serial in this implementation

    def handle_laser_lens_distance(self, command):
        logger.info(f"Handling laser lens distance: {command.values}")
        # Laser lens distance is not sent over serial in this implementation

    def handle_clear_no_fire_zone(self, command):
        logger.info("Handling clear no-fire zone")
        # Logic to clear no-fire zone would be implemented here

    def handle_return_home(self, command):
        logger.info("Handling return home")
        self.send_serial_message(0, 0, 0) # Example: stop motors and laser


def main(args=None):
    setup_logging()
    rclpy.init(args=args)
    node = CommandHandler()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if hasattr(node, 'ser') and node.ser.is_open:
            node.ser.close()
            logger.info("Serial port closed.")
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
