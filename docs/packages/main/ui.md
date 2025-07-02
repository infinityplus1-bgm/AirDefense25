# UI Node

## Overview

The UI Node provides a graphical user interface for controlling and monitoring the system.

## Subscribed Topics

- `/camera/image_raw` (`sensor_msgs/msg/Image`): Receives the raw camera image for display.
- `/results` (`interfaces/msg/Float32MultiArray2D`): Receives the detection and tracking results for display.

## Published Topics

- `/system/commands` (`interfaces/msg/Command`): Sends high-level commands to the system.
- `/system/mode` (`std_msgs/msg/Int32`): Sets the system mode.
- `/target_id` (`std_msgs/msg/Int32`): Sets the ID of the target to track.
- `/system/status` (`std_msgs/msg/Int32`): Toggles the system on and off.
