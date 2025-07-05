# src/main/main/config.py
"""
Configuration for the Air Defense System.
"""

# System Modes
MODE_MANUAL = "Manual"
MODE_PHASE_ONE = "Phase One"
MODE_PHASE_TWO = "Phase Two"
MODE_PHASE_THREE = "Phase Three"

SYSTEM_MODES = [MODE_MANUAL, MODE_PHASE_ONE, MODE_PHASE_TWO, MODE_PHASE_THREE]

MODE_MAP = {
    MODE_MANUAL: 0,
    MODE_PHASE_ONE: 1,
    MODE_PHASE_TWO: 2,
    MODE_PHASE_THREE: 3
}

# View Modes
VIEW_MODE_TRACKING = "Tracking"
VIEW_MODE_CAMERA = "Camera"

VIEW_MODES = [VIEW_MODE_TRACKING, VIEW_MODE_CAMERA]

# Movement Commands
MOVE_UP = "up"
MOVE_DOWN = "down"
MOVE_LEFT = "left"
MOVE_RIGHT = "right"

# Movement button values
MOVE_VALUE = 1 # degrees

# Serial Configuration
SERIAL_PORT = '/dev/ttyUSB0'
BAUD_RATE = 115200

# Model Paths
YOLO_MODEL_PATH = "best.pt"
IMAGE_PROCESSING_MODEL_PATH = "/home/teknofest/Documents/AirDefense25/assets/best.pt"

# ROS 2 Topic Names
TOPIC_HEALTH_TRACKING_NODE = '/health/tracking_node'
TOPIC_HEALTH_SERIAL_NODE = '/health/serial_node'
TOPIC_HEALTH_DETECTION_NODE = '/health/detection_node'
TOPIC_HEALTH_COMMAND_HANDLER_NODE = '/health/command_handler_node'
TOPIC_HEALTH_CAMERA_NODE = '/health/camera_node'
TOPIC_HEALTH_IMAGE_PROCESSING_NODE = '/health/image_processing_node'
TOPIC_HEALTH_SYSTEM = '/health/system'
TOPIC_CAMERA_IMAGE_RAW = 'camera/image_raw'
TOPIC_DETECTIONS = '/detections'
TOPIC_DETECTIONS_OVERLAY = '/detections/overlay'
TOPIC_RESULTS = '/results'
TOPIC_TRACKED_OBJECTS = 'tracked_objects'
TOPIC_SERIAL_COMMANDS = '/serial/commands'
TOPIC_SYSTEM_COMMANDS = '/system/commands'
TOPIC_SYSTEM_MODE = '/system/mode'
TOPIC_SYSTEM_STATUS = '/system/status'
TOPIC_TARGET_ID = '/target_id'
TOPIC_UI_COMMANDS = '/ui/commands'
TOPIC_LASER_COMMAND = 'laser/command'
TOPIC_MOTOR_PAN = 'motor/pan'
TOPIC_MOTOR_TILT = 'motor/tilt'

# ROS 2 Node Names
NODE_TRACKING = 'tracking'
NODE_DETECTIONS_OVERLAY_TEST = 'detections_overlay_test'
NODE_TRACKER_TEST = 'tracker_test'
NODE_YOLO_DETECTION = 'yolo_detection'
NODE_SERIAL = 'serial'
NODE_CAMERA_TEST = 'camera_test'
NODE_CAMERA = 'camera'
NODE_IMAGE_PROCESSING = 'image_processing'
NODE_SERIAL_TEST = 'serial_test'
NODE_COMMAND_HANDLER = 'command_handler'
NODE_HEALTH_MONITORING = 'health_monitoring_node'
NODE_UI = 'ui'
