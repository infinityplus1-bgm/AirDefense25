# Command Handler Node

## Overview

The Command Handler Node is the central decision-making unit of the system. It receives commands from the UI and tracking data from the Image Processing Node to determine the appropriate actions to take.

## Subscribed Topics

- `/system/status` (`std_msgs/msg/Int32`): Toggles the command handler on and off.
- `/system/mode` (`std_msgs/msg/String`): Sets the system mode (idle, manual, auto).
- `/ui/commands` (`std_msgs/msg/Int32MultiArray`): Receives commands from the UI.
- `/centroid_status` (`std_msgs/msg/Float32MultiArray`): Receives the centroid of the tracked object.
- `/tracked_object` (`std_msgs/msg/Float32MultiArray`): Receives the tracked object data.

## Published Topics

- `/motor/commands` (`std_msgs/msg/Int32MultiArray`): Sends commands to the motors.
- `/laser/commands` (`std_msgs/msg/Int32MultiArray`): Sends commands to the laser.
