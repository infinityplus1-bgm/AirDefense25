# Image Processing Node

## Overview

The Image Processing Node is responsible for performing object detection and tracking on the images received from the Camera Node.

## Subscribed Topics

- `/camera/image_raw` (`sensor_msgs/msg/Image`): The raw image frames from the camera.
- `/system/status` (`std_msgs/msg/Int32`): Toggles the image processing on and off.

## Published Topics

- `/results` (`interfaces/msg/Float32MultiArray2D`): A flattened 2D array containing the detection and tracking results. Each row in the array represents a detected object and contains the following information: `(x1, y1, x2, y2, object_id, confidence, class_id)`.
