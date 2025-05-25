import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String, Int32MultiArray
import numpy as np

class CommandHandler(Node):
    def __init__(self):
        super().__init__('command_handler')

        self.current_mode = "idle"
        
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
        
        self.get_logger().info('Command Handler node initialized')
    
    def mode_callback(self, msg):
        """
        Handle system mode changes
        """
        new_mode = msg.data
        
        # Log mode change if different
        if new_mode != self.current_mode:
            self.get_logger().info(f'System mode changed: {self.current_mode} -> {new_mode}')
            self.current_mode = new_mode
            
            # Process commands based on new mode
            self.process_commands()
    
    def ui_command_callback(self, msg):
        """
        Handle UI commands
        """
        # Store the latest UI commands
        self.latest_ui_commands = list(msg.data)
        
        # If in manual mode, process the command immediately
        if self.current_mode == "manual":
            self.process_commands()
    
    def centroid_callback(self, msg):
        """
        Handle centroid status data
        """
        # Store the latest centroid data
        self.latest_centroids = list(msg.data)
        
        # If in auto mode, might need to process this data
        if self.current_mode == "auto":
            self.process_auto_tracking()
    
    def tracked_object_callback(self, msg):
        """
        Handle tracked object data
        """
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
                self.get_logger().debug(f'Published motor commands: {self.latest_ui_commands}')
        
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
    rclpy.init(args=args)
    node = CommandHandler()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()