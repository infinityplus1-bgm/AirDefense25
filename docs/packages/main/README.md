# Package: main

## Overview

This is the **core package** for the AirDefense2025 competition robot. It contains the essential nodes required to run the air defense system on the **real hardware**.

This package integrates sensor data processing, target detection, tracking, command handling, user interface updates, and hardware interfacing via a serial bridge.

## Architecture

The system is composed of the following ROS 2 nodes:

- **[Camera Node](./camera.md):** Captures images from the camera and publishes them.
- **[Image Processing Node](./image_processing.md):** Performs object detection and tracking on the images.
- **[Command Handler Node](./command_handler.md):** Manages system state and sends commands to the hardware.
- **[Serial Node](./serial.md):** Communicates with the hardware (motors, laser).
- **[UI Node](./ui.md):** Provides a graphical user interface for system control and monitoring.

## Topics

The nodes communicate using the following topics:

- `/camera/image_raw`: Raw camera images.
- `/results`: Detection and tracking results.
- `/system/status`: System status (enabled/disabled).
- `/system/mode`: System mode (idle, manual, auto).
- `/ui/commands`: Commands from the UI.
- `/motor/commands`: Commands for the motors.
- `/laser/commands`: Commands for the laser.
- `/target_id`: The ID of the selected target.
