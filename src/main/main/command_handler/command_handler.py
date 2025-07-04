import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32
from interfaces.msg import Command, Float32MultiArray2D
import logging
from main.utils.logging_config import setup_logging
from main.utils import commands as cmd
from main import config
import numpy as np


logger = logging.getLogger(__name__)

class CommandHandler(Node):
    def __init__(self):
        super().__init__(config.NODE_COMMAND_HANDLER)

        self.current_mode = config.MODE_MANUAL
        self.system_enabled = False
        self.latest_tracking_results = None

        # Subscribers
        self.status_sub = self.create_subscription(Int32, config.TOPIC_SYSTEM_STATUS, self.status_callback, 10)
        self.mode_sub = self.create_subscription(String, config.TOPIC_SYSTEM_MODE, self.mode_callback, 10)
        self.ui_command_sub = self.create_subscription(Command, config.TOPIC_UI_COMMANDS, self.ui_command_callback, 10)
        self.tracking_sub = self.create_subscription(Float32MultiArray2D, config.TOPIC_RESULTS, self.tracking_results_callback, 10)
        
        # Publisher for serial commands
        self.serial_command_publisher = self.create_publisher(Command, config.TOPIC_SERIAL_COMMANDS, 10)
        
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

    def handle_set_no_fire_zone(self, command):
        logger.info(f"Handling set no-fire zone: {command.values}")
        self.serial_command_publisher.publish(command)

    def handle_laser_power_control(self, command):
        logger.info(f"Handling laser power control: {command.values}")
        self.serial_command_publisher.publish(command)

    def handle_pid_values(self, command):
        logger.info(f"Handling PID values: {command.values}")
        self.serial_command_publisher.publish(command)

    def handle_laser_lens_distance(self, command):
        logger.info(f"Handling laser lens distance: {command.values}")
        self.serial_command_publisher.publish(command)

    def handle_clear_no_fire_zone(self, command):
        logger.info("Handling clear no-fire zone")
        self.serial_command_publisher.publish(command)

    def handle_return_home(self, command):
        logger.info("Handling return home")
        self.serial_command_publisher.publish(command)


def main(args=None):
    setup_logging()
    rclpy.init(args=args)
    node = CommandHandler()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
