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
import threading # Import threading for serial read loop

logger = logging.getLogger(__name__)

class CommandHandler(Node):
    def __init__(self):
        super().__init__(config.NODE_COMMAND_HANDLER)


        self.current_mode = config.MODE_MANUAL_ID
        self.system_enabled = False
        self.latest_tracking_results = None

        # Define a constant for the number of steps for each manual movement command
        # Adjust this value based on how much you want the motors to move per command
        self.MOVEMENT_STEPS_PAN = 275
        # self.MOVEMENT_STEPS_PAN = 5000

        self.MOVEMENT_STEPS_TILT = 185

        self.LASER_POWER = 0


        self.PAN_STEPS_TAKEN = 0
        self.TILT_STEPS_TAKEN = 0


        self.SELECTED_TARGET_ID = None


        self.IS_SHOOTING = False


        # no fire zone
        self.NO_FIRE_ZONE = None

        # Serial communication setup
        self.serial_port = config.SERIAL_PORT
        self.baud_rate = config.BAUD_RATE
        
        try:
            self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=0.1) # Shorter timeout for non-blocking read
            logger.info(f"Serial port {self.serial_port} opened successfully at {self.baud_rate} baud.")
            # Start a separate thread for reading serial data
            self.serial_read_thread = threading.Thread(target=self.read_serial_data, daemon=True)
            self.serial_read_thread.start()
            logger.info("Serial read thread started.")

        except serial.SerialException as e:
            logger.error(f"Error opening serial port: {e}")
            rclpy.shutdown()
            return

        # Subscribers
        self.status_sub = self.create_subscription(Int32, config.TOPIC_SYSTEM_STATUS, self.status_callback, 10)
        self.mode_sub = self.create_subscription(Int32, config.TOPIC_SYSTEM_MODE, self.mode_callback, 10)

        self.ui_command_sub = self.create_subscription(Command, config.TOPIC_SYSTEM_COMMANDS, self.ui_command_callback, 10)
        self.tracking_sub = self.create_subscription(Float32MultiArray2D, config.TOPIC_RESULTS, self.tracking_results_callback, 10)
        
        # Health publisher
        self.health_publisher = self.create_publisher(String, config.TOPIC_HEALTH_COMMAND_HANDLER_NODE, 10)
        self.health_timer = self.create_timer(1, self.publish_health_status)
        
        logger.info('Command Handler node initialized')

    def publish_health_status(self):
      
        """Publishes the health status of the node."""
        msg = String()
        msg.data = 'healthy'
        self.health_publisher.publish(msg)

    def status_callback(self, msg):
        """Callback for system status updates."""

        self.system_enabled = bool(msg.data)
        logger.info(f"System status updated: {'Enabled' if self.system_enabled else 'Disabled'}")

    def mode_callback(self, msg):
        """Callback for system mode updates."""
        logger.info("received change mode")

        new_mode = msg.data
        if new_mode != self.current_mode:
            logger.info(f'System mode changed: {self.current_mode} -> {new_mode}')
            self.current_mode = new_mode

    def tracking_results_callback(self, msg : Float32MultiArray2D):
        """Callback for tracking results."""
        # Reshape the received tracking results into a 2D numpy array
        self.latest_tracking_results = np.array(msg.data, dtype=np.float32).reshape(msg.rows, msg.cols)
        
        # logger.info(self.latest_tracking_results)
        if self.current_mode == config.MODE_MANUAL_ID :
            return 
        elif self.current_mode == config.MODE_PHASE_ONE_ID and self.SELECTED_TARGET_ID is None:
            return
        elif self.current_mode == config.MODE_PHASE_TWO_ID and self.SELECTED_TARGET_ID is None:
            try:
                self.SELECTED_TARGET_ID = self.latest_tracking_results[0,4]
            except:
                self.SELECTED_TARGET_ID = None
                return
            # we should run a function for selecting a target
        elif self.current_mode == config.MODE_PHASE_THREE_ID:
            return
        

        # logger.warning("going to target")
        # once we are sure that there is a target locked we need to get the row for that target
        mask = self.latest_tracking_results[:, 4] == self.SELECTED_TARGET_ID

        logger.warning(mask)

        locked_target_row = self.latest_tracking_results[mask]

        if len(locked_target_row) == 0:
            logger.warning("empty mask")
            self.SELECTED_TARGET_ID = None
            self.IS_SHOOTING = False
            return


        logger.warning(locked_target_row)
        # TODO : 1. check if the target is in the kill-zone if so issue "shoot" command
        # Zoom 3
        # Distance 5m
        # 1cm --> 4.58 pixel
        p = 4.58

        # get the center of of the object detected
        
        baloon_center = np.array(((locked_target_row[0,0] + locked_target_row[0,2]) // 2 , (locked_target_row[0,1] + locked_target_row[0,3]) // 2))
            # self.SELECTED_TARGET_ID = None
            # self.IS_SHOOTING = False
            # return
        logger.warning(baloon_center)

        diff = baloon_center - config.LASER_CENTER
        distance = np.linalg.norm(diff)

        
        if distance <= p * 3 and not self.IS_SHOOTING:
            self.send_serial_message(self.LASER_POWER,0,0)
            self.IS_SHOOTING = True
            return
        # TODO : 2. calculate the steps that we need to reach that target and issue them through serial to the motor
        
        # calculate the steps that we need 
        distance_x = (diff[0] / p) / 100
        distance_y = diff[1] / p / 100

        # steps_pan = int(np.rad2deg(np.arctan(distance_x / 5)) / 0.009)
        # steps_tilt = int(np.rad2deg(np.arctan(distance_y / 5)) / 0.009)
        # give 20% of the steps
        steps_pan = np.rad2deg(np.arctan(distance_x / 5)) / 0.009 // 2
        steps_tilt = np.rad2deg(np.arctan(distance_y / 5)) / 0.009 
        logger.info(f"distance_x : {distance_x} , distance_y : {distance_y}")
        logger.info(f"steps_pan : {steps_pan} , steps_tilt : {steps_tilt}")
        # logger.info(f"")
        self.send_serial_message(0,-int(steps_tilt),int(steps_pan))



    def ui_command_callback(self, msg : Command):
        """Callback for UI commands. Dispatches commands to appropriate handlers."""

        if not self.system_enabled:
            logger.warning("System is disabled. Ignoring command.")
            return
        

        logger.info(f"Received UI command: {msg.id}")

        # Dictionary mapping command IDs to their respective handler functions
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
            cmd.CMD_CLEAR_TARGET: self.handle_clear_target,

        }

        handler = command_handlers.get(msg.id)
        if handler:

            handler(msg) # Call the appropriate handler function
        else:
            logger.warning(f"No handler found for command ID: {msg.id}")

    def handle_clear_target(self, command):
        """Handles clearing the selected target."""
        logger.info(f"Handling clear selected target")
        self.SELECTED_TARGET_ID = None
    
    def handle_movement(self, command : Command):
        """Handles movement commands (up, down, right, left)."""
        if self.current_mode != config.MODE_MANUAL_ID:
            logger.warning("Movement command received, but not in manual mode. Ignoring.")
            return
        
        logger.info(f"Handling movement: {cmd.COMMAND_DESCRIPTIONS.get(command.id, 'Unknown Movement Command')}")

        motor_pan_steps = 0
        motor_tilt_steps = 0

        # Determine steps based on the movement command
        if command.id == cmd.CMD_MOVE_RIGHT:
            motor_pan_steps = self.MOVEMENT_STEPS_PAN
        elif command.id == cmd.CMD_MOVE_LEFT:
            motor_pan_steps = -self.MOVEMENT_STEPS_PAN
        elif command.id == cmd.CMD_MOVE_UP:
            motor_tilt_steps = self.MOVEMENT_STEPS_TILT
        elif command.id == cmd.CMD_MOVE_DOWN:
            motor_tilt_steps = -self.MOVEMENT_STEPS_TILT
        
        self.PAN_STEPS_TAKEN += motor_pan_steps
        self.TILT_STEPS_TAKEN += motor_tilt_steps

        # Send the movement command over serial (laser PWM is 0 for movement)
        self.send_serial_message(0,motor_tilt_steps, motor_pan_steps )

    def handle_shoot(self, command):
        """Handles the shoot command."""
        if self.current_mode not in (config.MODE_MANUAL_ID, config.MODE_PHASE_ONE_ID):
            logger.warning(f"Shoot command received, but not in an appropriate mode. Current mode: {self.current_mode}. Ignoring.")
            return
        if self.NO_FIRE_ZONE:
           current_degrees = self.PAN_STEPS_TAKEN * 0.009 # 0.009 is degrees per step
           if (self.NO_FIRE_ZONE[0] <= current_degrees <= self.NO_FIRE_ZONE[1]):
            logger.warning(f"Shoot command received, but inside no-fire zone. Ignoring.")
            return
        logger.info("Handling shoot command")
        # Example: send a specific serial command to trigger shooting
        # This would typically involve setting laser_pwm to a high value for a short duration
        # For now, let's assume a full power burst for demonstration
        self.send_serial_message(self.LASER_POWER, 0, 0) # Full laser power, no motor movement

    def handle_select_target(self, command):
        """Handles the select target command."""
        if self.current_mode != config.MODE_PHASE_ONE_ID:
            logger.warning(f"Select target command received, but not in phase 1. Current mode: {self.current_mode}. Ignoring.")
            return
        logger.info(f"Handling select target with values: {command.values}")

        self.SELECTED_TARGET_ID = int(command.values[0])

    def send_serial_message(self, laser_pwm: int, motor_pan: int, motor_tilt: int):
        """
        Packs and sends serial data to the Arduino/ESP32.
        
        The message format is:
        - Byte 0: Laser PWM (unsigned 8-bit)
        - Bytes 1-2: Motor A (Pan) steps (signed 16-bit)
        - Bytes 3-4: Motor B (Tilt) steps (signed 16-bit)
        """
        if not self.system_enabled:
            logger.warning("System is disabled. Not sending serial message.")
            return
        
        try:
            # Pack the data into a byte string using little-endian format ('<Bhh')
            # B: unsigned char (1 byte) for laser_pwm
            # h: short (2 bytes) for motor_pan (signed 16-bit)
            # h: short (2 bytes) for motor_tilt (signed 16-bit)
            packed_data = struct.pack('<Bhh', laser_pwm, motor_pan, motor_tilt)
            self.ser.write(packed_data)
            logger.info(f"Sent serial: Laser PWM: {laser_pwm}, Motor Pan: {motor_pan}, Motor Tilt: {motor_tilt} (Bytes: {packed_data.hex()})")

        except serial.SerialException as e:
            logger.error(f"Error sending data over serial: {e}")
        except struct.error as e:
            logger.error(f"Error packing data: {e}. Check data types and ranges.")

        except Exception as e:
            logger.error(f"An unexpected error occurred while sending serial data: {e}")

    def read_serial_data(self):
        """
        Continuously reads data from the serial port in a separate thread.
        Logs the received data.
        """
        logger.info("Serial read thread started and listening for incoming data...")
        while rclpy.ok() and self.ser.is_open:
            try:
                if self.ser.in_waiting > 0:
                    # Read a line from the serial port.
                    # The Arduino sketch sends data followed by a newline.
                    received_line = self.ser.readline()
                    
                    try:
                        # Decode the bytes to a string, stripping whitespace
                        decoded_line = received_line.decode('utf-8').strip()
                        if decoded_line: # Only log if the line is not empty
                            logger.info(f"Received from serial: {decoded_line}")
                    except UnicodeDecodeError:
                        logger.warning(f"Could not decode serial data: {received_line.hex()}")
                # Small delay to prevent busy-waiting and allow other threads to run
                # This timeout is also set in the serial.Serial constructor
                # time.sleep(0.01) 
            except serial.SerialException as e:
                logger.error(f"Serial port error during read: {e}")
                break # Exit loop on serial error
            except Exception as e:
                logger.error(f"An unexpected error occurred in serial read thread: {e}")
                break # Exit loop on unexpected error
        logger.info("Serial read thread stopped.")


    def handle_set_no_fire_zone(self, command):
        """Handles setting a no-fire zone."""
        logger.info(f"Handling set no-fire zone with values: {command.values}")
        # Logic to handle no-fire zone would be implemented here.
        self.NO_FIRE_ZONE = command.values


    def handle_laser_power_control(self, command):
        """Handles laser power control."""
        logger.info(f"Handling laser power control with values: {command.values}")
        power = int(command.values[0])
        # Ensure power is within the valid range [0, 255]
        if not (0 <= power <= 100):
            logger.warning(f"Laser PWM value out of range (0-255): {power}. Clamping.")
            return
        self.LASER_POWER = int(255 * (power / 100))
        # self.send_serial_message(power, 0, 0) # Send laser power, no motor movement

    def handle_pid_values(self, command):
        """Handles PID value updates."""
        logger.info(f"Handling PID values: {command.values}")
        # PID values are typically used for internal control loops on the ESP32
        # and might be sent over serial if the ESP32 needs to update its PID constants.
        # Not implemented for serial transmission in this example.

    def handle_laser_lens_distance(self, command):
        """Handles laser lens distance adjustments."""
        logger.info(f"Handling laser lens distance: {command.values}")
        # Laser lens distance is not sent over serial in this current implementation.
        # If needed, the serial protocol would need to be extended.

    def handle_clear_no_fire_zone(self, command):
        """Handles clearing the no-fire zone."""
        logger.info("Handling clear no-fire zone")
        
        self.NO_FIRE_ZONE = None

    def handle_return_home(self, command):
        """Handles the return home command."""
        logger.info("Handling return home command")
        # Example: send a command to stop all motors and reset laser power

        self.send_serial_message(0, -self.TILT_STEPS_TAKEN, -self.PAN_STEPS_TAKEN) # Stop motors and turn off laser
        self.PAN_STEPS_TAKEN = 0
        self.TILT_STEPS_TAKEN = 0

def main(args=None):
    setup_logging() # Initialize logging for the node
    rclpy.init(args=args) # Initialize ROS 2
    node = CommandHandler() # Create an instance of the CommandHandler node
    try:
        rclpy.spin(node) # Keep the node alive and process callbacks
    except KeyboardInterrupt:
        logger.info("Node stopped by KeyboardInterrupt.")
    finally:
        # Ensure the serial port is closed and the node is destroyed gracefully
        if hasattr(node, 'ser') and node.ser.is_open:
            node.ser.close()
            logger.info("Serial port closed.")
        node.destroy_node()
        rclpy.shutdown() # Shutdown ROS 2

if __name__ == '__main__':
    main()

