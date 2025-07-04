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
