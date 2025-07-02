import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String, Int32MultiArray, Int32
import numpy as np
import logging
from main.utils.logging_config import setup_logging

logger = logging.getLogger(__name__)

class CommandHandler(Node):
    def __init__(self):
        super().__init__('command_handler')

        self.current_mode = "idle"
        self.system_enabled = False

        # Subscribe to system status
        self.status_sub = self.create_subscription(Int32, '/system/status', self.status_callback, 10)
        
        # Subscribe to system mode
        self.mode_sub = self.create_subscription(String,'/system/mode',self.mode_callback,10)
        
        # Subscribe to UI commands
        self.ui_command_sub = self.create_subscription(Int32MultiArray,'/ui/commands',self.ui_command_callback,10)
        
        # Subscribe to centroid status from tracker
        self.centroid_sub = self.create_subscription(Float32MultiArray,'/centroid_status',self.centroid_callback,10)
        
        # Subscribe to tracked objects from tracker
        self.tracked_object_sub = self.create_subscription(Float32MultiArray,'/tracked_object',self.tracked_object_callback,10)
        
        # Publisher for motor commands (stepper motor steps)
        self.motor_publisher = self.create_publisher(Int32MultiArray,'/motor/commands',10)
        
        # Publisher for laser commands
        self.laser_publisher = self.create_publisher(Int32MultiArray,'/laser/commands',10)
        
        # Store the latest UI commands
        self.latest_ui_commands = []
        
        # Store the latest centroid data
        self.latest_centroids = []
        
        # Store the latest tracked objects
        self.latest_tracked_objects = []

        self.health_publisher = self.create_publisher(String, '/health/command_handler_node', 10)
        self.health_timer = self.create_timer(1, self.publish_health_status)
        
        logger.info('Command Handler node initialized')

    def publish_health_status(self):
        msg = String()
        msg.data = 'healthy'
        self.health_publisher.publish(msg)

    def status_callback(self, msg):
        """
        Handle system status changes
        """
        self.system_enabled = bool(msg.data)
        if not self.system_enabled:
            logger.info('System disabled. Command Handler inactive.')
        else:
            logger.info('System enabled. Command Handler active.')

    def mode_callback(self, msg):
        """
        Handle system mode changes
        """
        if not self.system_enabled:
            return
        new_mode = msg.data
        
        # Log mode change if different
        if new_mode != self.current_mode:
            logger.info(f'System mode changed: {self.current_mode} -> {new_mode}')
            self.current_mode = new_mode
            
            # Process commands based on new mode
            self.process_commands()
    
    def ui_command_callback(self, msg):
        """
        Handle UI commands
        """
        if not self.system_enabled:
            return
        # Store the latest UI commands
        self.latest_ui_commands = list(msg.data)
        
        # If in manual mode, process the command immediately
        if self.current_mode == "manual":
            self.process_commands()
    
    def centroid_callback(self, msg):
        """
        Handle centroid status data
        """
        if not self.system_enabled:
            return
        # Store the latest centroid data
        self.latest_centroids = list(msg.data)
        
        # If in auto mode, might need to process this data
        if self.current_mode == "auto":
            self.process_auto_tracking()
    
    def tracked_object_callback(self, msg):
        """
        Handle tracked object data
        """
        if not self.system_enabled:
            return
        # Store the latest tracked objects
        self.latest_tracked_objects = list(msg.data)
    
    def process_commands(self):
        """
        Process commands based on current mode
        """
        if self.current_mode == "manual":
            # In manual mode, forward UI commands to motors
            if self.latest_ui_commands:
                motor_msg = Int32MultiArray()
                motor_msg.data = self.latest_ui_commands
                self.motor_publisher.publish(motor_msg)
                logger.debug(f'Published motor commands: {self.latest_ui_commands}')
        
        elif self.current_mode == "auto":
            # In auto mode, processing is handled by process_auto_tracking
            pass
        
        else:
            # In idle mode or any other mode, don't send commands
            pass
    
    def process_auto_tracking(self):
        """
        Process tracking data for auto mode
        """
        # Auto tracking logic would go here   
        # For now, this is a placeholder
        pass

def main(args=None):
    setup_logging()
    rclpy.init(args=args)
    node = CommandHandler()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
