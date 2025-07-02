# Serial Node

## Overview

The Serial Node acts as a bridge between the ROS 2 system and the hardware, communicating with microcontrollers to control the motors and laser.

## Subscribed Topics

- `/system/status` (`std_msgs/msg/Int32`): Toggles the serial node on and off.
- `laser/command` (`std_msgs/msg/UInt8`): Receives the laser PWM value.
- `motor/pan` (`std_msgs/msg/Int16`): Receives the pan motor steps.
- `motor/tilt` (`std_msgs/msg/Int16`): Receives the tilt motor steps.

## Published Topics

None

## Parameters

- `serial_port`: The serial port to use for communication.
- `baud_rate`: The baud rate for serial communication.
