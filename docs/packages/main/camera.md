# Camera Node

## Overview

The Camera Node is responsible for capturing images from a physical camera or a video file and publishing them to the ROS 2 network.

## Subscribed Topics

- `/system/status` (`std_msgs/msg/Int32`): Toggles the camera on and off.

## Published Topics

- `/camera/image_raw` (`sensor_msgs/msg/Image`): The raw image frames from the camera.

## Parameters

- `video_path`: The path to the video file or the index of the webcam.
- `timer_period`: The period at which to publish images.
