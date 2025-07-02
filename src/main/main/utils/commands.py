"""
Centralized command definitions for the Air Defense System.
This file is used by the UI and other ROS nodes to ensure consistency.
"""

# Command IDs
CMD_MOVE_RIGHT = 0
CMD_MOVE_LEFT = 1
CMD_MOVE_UP = 2
CMD_MOVE_DOWN = 3
CMD_SHOOT = 4
CMD_SELECT_TARGET = 5
CMD_SET_NO_FIRE_ZONE = 6
CMD_LASER_POWER_CONTROL = 7
CMD_PID_VALUES = 8
CMD_LASER_LENS_DISTANCE = 9
CMD_CLEAR_NO_FIRE_ZONE = 10
CMD_RETURN_HOME = 11

# A dictionary for easy lookup, if needed
COMMANDS = {
    "MOVE_RIGHT": CMD_MOVE_RIGHT,
    "MOVE_LEFT": CMD_MOVE_LEFT,
    "MOVE_UP": CMD_MOVE_UP,
    "MOVE_DOWN": CMD_MOVE_DOWN,
    "SHOOT": CMD_SHOOT,
    "SELECT_TARGET": CMD_SELECT_TARGET,
    "SET_NO_FIRE_ZONE": CMD_SET_NO_FIRE_ZONE,
    "LASER_POWER_CONTROL": CMD_LASER_POWER_CONTROL,
    "PID_VALUES": CMD_PID_VALUES,
    "LASER_LENS_DISTANCE": CMD_LASER_LENS_DISTANCE,
    "CLEAR_NO_FIRE_ZONE": CMD_CLEAR_NO_FIRE_ZONE,
    "RETURN_HOME": CMD_RETURN_HOME,
}

# Descriptions for UI or logging purposes
COMMAND_DESCRIPTIONS = {
    CMD_MOVE_RIGHT: "Move right by X degrees (pan)",
    CMD_MOVE_LEFT: "Move left by X degrees (pan)",
    CMD_MOVE_UP: "Move up by X degrees (tilt)",
    CMD_MOVE_DOWN: "Move down by X degrees (tilt)",
    CMD_SHOOT: "Activate the laser for 1 second",
    CMD_SELECT_TARGET: "Select a target by its tracker ID",
    CMD_SET_NO_FIRE_ZONE: "Set a no-fire zone with min and max degrees",
    CMD_LASER_POWER_CONTROL: "Set laser power to X%",
    CMD_PID_VALUES: "Set PID values for Pan and Tilt",
    CMD_LASER_LENS_DISTANCE: "Set laser lens distance in cm",
    CMD_CLEAR_NO_FIRE_ZONE: "Clear any set no-fire zones",
    CMD_RETURN_HOME: "Return to the home position",
}
