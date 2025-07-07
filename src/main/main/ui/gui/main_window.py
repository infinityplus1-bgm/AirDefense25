import logging
import threading
import rclpy
from PyQt5.QtWidgets import QMainWindow

from main.utils.logging_config import setup_logging
from main.utils.commands import (
    CMD_MOVE_UP, CMD_MOVE_DOWN, CMD_MOVE_LEFT, CMD_MOVE_RIGHT,
    CMD_SHOOT, CMD_SELECT_TARGET, CMD_SET_NO_FIRE_ZONE,
    CMD_LASER_POWER_CONTROL, CMD_PID_VALUES, CMD_LASER_LENS_DISTANCE,
    CMD_CLEAR_NO_FIRE_ZONE, CMD_RETURN_HOME
)
from .styles import STYLESHEET
from .ros_connector import ROSConnector
from .ui_manager import UIManager
from main.config import MODE_MAP

logger = logging.getLogger(__name__)

class AirDefenseGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        setup_logging()
        logger.info("GUI Initializing...")

        self.system_active = False
        self.ros_connector = ROSConnector()
        self.ui_manager = UIManager(self)

        self._start_ros_spin()
        self._configure_ui()
        self._connect_signals()

    def _start_ros_spin(self):
        ros_thread = threading.Thread(target=rclpy.spin, args=(self.ros_connector,), daemon=True)
        ros_thread.start()

    def _configure_ui(self):
        self.setWindowTitle('Infinity + 1 GUI')
        self.resize(1100, 800)
        self.setStyleSheet(STYLESHEET)

    def _connect_signals(self):
        self.ui_manager.connect_signals(self)
        self.ros_connector.image_received.connect(self.ui_manager.image_view.update_image)
        self.ros_connector.target_list_updated.connect(self.ui_manager.target_selector.update_targets)
        self.ros_connector.system_status_updated.connect(self.ui_manager.health_check_display.update_status)


    # --- System State Handlers ---
    def _start_system(self):
        if self.system_active:
            return
        logger.warning("SYSTEM STARTING...")
        self.system_active = True
        self.ros_connector.publish_status(1)
        self.ui_manager.system_control_buttons.set_buttons_state(self.system_active)

    def _stop_system(self):
        if not self.system_active:
            return
        logger.warning("SYSTEM STOPPING...")
        self.system_active = False
        self.ros_connector.publish_status(0)
        self.ui_manager.system_control_buttons.set_buttons_state(self.system_active)

    # --- Component Signal Handlers ---
    def _handle_view_mode_change(self, view_mode: str):
        logger.info(f"View mode changed to: {view_mode}")
        self.ros_connector.set_view_mode(view_mode)

    def _handle_return_home(self):
        logger.info("Return Home button clicked.")
        self.ros_connector.publish_command(CMD_RETURN_HOME, [])

    def _handle_no_fire_zone(self, zone: dict):
        logger.info(f"No-Fire Zone set: {zone}")
        self.ros_connector.publish_command(CMD_SET_NO_FIRE_ZONE, [zone['lower_deg'], zone['upper_deg']])

    def _handle_clear_no_fire_zone(self):
        logger.info("No-Fire Zone cleared.")
        self.ros_connector.publish_command(CMD_CLEAR_NO_FIRE_ZONE, [])

    def _handle_mode_change(self, mode_name: str):
        logger.info(f"Mode: {mode_name}")
        self.ros_connector.publish_mode(MODE_MAP[mode_name])

    def _handle_target_selection(self, target_id: int):
        logger.info(f"Selected Target ID: {target_id}")
        self.ros_connector.publish_command(CMD_SELECT_TARGET, [float(target_id)])

    def _handle_target_cleared(self):
        logger.info("Target selection cleared.")
        # Optionally, publish a special value like -1 to indicate no target
        self.ros_connector.publish_target_id(-1)

    def _handle_movement_command(self, direction: str):
        direction_map = {
            "up": CMD_MOVE_UP,
            "down": CMD_MOVE_DOWN,
            "left": CMD_MOVE_LEFT,
            "right": CMD_MOVE_RIGHT
        }
        logger.debug(f"Move: {direction}")
        # Assuming movement commands now take a value, sending 1.0 as a placeholder
        self.ros_connector.publish_command(direction_map[direction], [1.0]) # FIXME: change this to whatever default value we want or take it from a config file

    def _handle_shoot_command(self):
        if not self.system_active:
            logger.error("Cannot SHOOT: System INACTIVE.")
            return
        logger.critical("ACTION: FIRING LASER")
        self.ros_connector.publish_command(CMD_SHOOT, [])

    def _handle_laser_target_power_change(self, power: int):
        logger.info(f"Laser Target: {power}%")
        self.ros_connector.publish_command(CMD_LASER_POWER_CONTROL, [float(power)])

    def _handle_laser_lens_change(self, value: int):
        logger.info(f"Laser Lens: {value}mm")
        self.ros_connector.publish_command(CMD_LASER_LENS_DISTANCE, [float(value)])

    def _handle_pid_settings_applied(self, pP_s, pI_s, pD_s, tP_s, tI_s, tD_s):
        logger.info(f"Pan PID set: P={pP_s}, I={pI_s}, D={pD_s}")
        logger.info(f"Tilt PID set: P={tP_s}, I={tI_s}, D={tD_s}")
        self.ros_connector.publish_command(CMD_PID_VALUES, [pP_s, pI_s, pD_s, tP_s, tI_s, tD_s])


    # --- ROS Connector Signal Handlers ---
    def _update_pid_from_ros(self, motor: str, p: float, i: float, d: float):
        logger.debug(
            f"ROS PID Update for {motor}: P{p:.2f} I{i:.2f} D{d:.2f}"
        )

    def _update_health_from_ros(self, status: dict):
        logger.debug(f"ROS Health Update: {status}")

    def closeEvent(self, event):
        logger.info("GUI Shutting down...")
        self.ros_connector.shutdown()
        event.accept()
