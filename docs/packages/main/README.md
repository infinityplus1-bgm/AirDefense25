# Package: main

## Overview

This is the **core package** for the AirDefense2025 competition robot. It contains the essential nodes, launch files, and configurations required to run the air defense system on the **real hardware**, based on the system diagram provided.

This package integrates sensor data processing, target detection, tracking, command handling, user interface updates, system monitoring, and hardware interfacing via a serial bridge.

## Dependencies

*   ROS2 Humble
*   Specific hardware drivers/SDKs (Camera, Serial device)
*   Libraries like OpenCV (`libopencv-dev`), potentially YOLO model libraries.
*   **A custom interface package** (e.g., `air_defense_interfaces`) defining message and service types for topics like `/detections`, `/tracked_object`, `/system/health`, etc. **This package must be created and listed as a dependency.**
*   Standard ROS2 message packages: `sensor_msgs`, `std_msgs`, `geometry_msgs`.

## Nodes

This section details the ROS2 nodes identified in the system diagram. *Note: Executable names are defined in the package's `setup.py` (for Python) or `CMakeLists.txt` (for C++) and may differ from these assumptions.*

---

### Camera Node

*   **Purpose:** Captures images from the physical camera hardware and publishes them for processing.
*   **Executable Name (Assumed):** `camera_node` or `camera_driver`
*   **Subscribed Topics:** None (Interfaces with hardware).
*   **Published Topics:**
    *   `/camera/image_raw` (`sensor_msgs/msg/Image`)
*   **Parameters (Potential):** Camera ID/path, resolution, frame rate, calibration parameters.

---

### Yolo_detection Node

*   **Purpose:** Performs object detection (likely using YOLO) on incoming raw images to identify potential targets. Publishes detection results and optionally an overlay image for visualization.
*   **Executable Name (Assumed):** `yolo_node`
*   **Subscribed Topics:**
    *   `/camera/image_raw` (`sensor_msgs/msg/Image`)
*   **Published Topics:**
    *   `/detections` (`std_msgs/msg/Float32MultiArray`)
    *   `/detections_overlay` (`sensor_msgs/msg/Image`)
*   **Parameters (Potential):** Model path.

---

### Tracking Node

*   **Purpose:** Takes raw detections and performs object tracking over time. Assigns IDs to targets and estimates their state (e.g., position, velocity).
*   **Executable Name (Assumed):** `tracking_node`
*   **Subscribed Topics:**
    *   `/detections` (`std_msgs/msg/Float32MultiArray`)
*   **Published Topics:**
    *   `/centroid_status` (Custom message, e.g., `air_defense_interfaces/msg/CentroidStatus`)
    *   `/tracked_object` (Custom message, e.g., `air_defense_interfaces/msg/TrackedObject`)
*   **Parameters (Potential):** Tracking algorithm (e.g., Kalman filter, SORT), max missed frames, association thresholds.

---

### Command_handler Node

*   **Purpose:** Acts as the central decision-making unit. Takes tracked object information, commands from the UI, and potentially system status to determine appropriate actions, such as aiming motors and firing lasers. Sets the overall system mode.
*   **Executable Name (Assumed):** `command_handler_node`
*   **Subscribed Topics:**
    *   `/centroid_status` (Custom message)
    *   `/tracked_object` (Custom message)
    *   `/ui/commands` (Custom message or `std_msgs/msg/String`)
*   **Published Topics:**
    *   `/motor/commands` (Custom message or `geometry_msgs/msg/Twist`)
    *   `/laser/commands` (Custom message or `std_msgs/msg/Bool`)
    *   `/system/mode` (`std_msgs/msg/String` or custom)
*   **Parameters (Potential):** Engagement logic, targeting priorities, safety constraints.

---

### Serial Node

*   **Purpose:** Bridges communication between the ROS2 system and hardware components connected via a serial interface (e.g., microcontrollers controlling motors and lasers). Translates ROS commands to serial messages and serial feedback to ROS topics.
*   **Executable Name (Assumed):** `serial_bridge_node`
*   **Subscribed Topics:**
    *   `/motor/commands` (Custom message or `geometry_msgs/msg/Twist`)
    *   `/laser/commands` (Custom message or `std_msgs/msg/Bool`)
*   **Published Topics:**
    *   `/motor/feedback` (Custom message or `sensor_msgs/msg/JointState`)
    *   `/laser/status` (Custom message or `std_msgs/msg/Bool`)
*   **Parameters (Potential):** Serial port path, baud rate, communication protocol details.

---

### System_monitor Node

*   **Purpose:** Monitors the feedback from hardware (via the Serial node) and potentially other system metrics to assess the overall health and status of the robot.
*   **Executable Name (Assumed):** `system_monitor_node`
*   **Subscribed Topics:**
    *   `/motor/feedback` (Custom message or `sensor_msgs/msg/JointState`)
    *   `/laser/status` (Custom message or `std_msgs/msg/Bool`)
*   **Published Topics:**
    *   `/system/health` (Custom message, e.g., `air_defense_interfaces/msg/SystemHealth`)
*   **Parameters (Potential):** Health check thresholds, monitoring frequency.

---

### UI Node

*   **Purpose:** Provides a graphical user interface for operators. Displays system status (health, mode), visual feedback (overlay image), and allows sending high-level commands.
*   **Executable Name (Assumed):** `ui_node` (Likely defined in the `ui` package, not `main`)
*   **Subscribed Topics:**
    *   `/detections_overlay` (`sensor_msgs/msg/Image`)
    *   `/system/health` (Custom message)
    *   `/system/mode` (`std_msgs/msg/String` or custom)
*   **Published Topics:**
    *   `/ui/commands` (Custom message or `std_msgs/msg/String`)
*   **Parameters (Potential):** UI layout configuration.

---

## Topics

This section details the ROS2 topics identified in the system diagram. *Note: Custom message types must be defined in a separate interface package (e.g., `air_defense_interfaces`). Standard messages should be preferred where applicable.*

---

*   **`/camera/image_raw`**
    *   **Message Type:** `sensor_msgs/msg/Image`
    *   **Description:** Raw image frames captured from the camera.
    *   **Publisher(s):** `Camera Node`
    *   **Subscriber(s):** `Yolo_detection Node`

*   **`/detections`**
    *   **Message Type:** `std_msgs/msg/Float32MultiArray` 
    *   **Description:** a flattened 1d array that contain `(x1,y1,x2,y2,confidence)`.
    *   **Publisher(s):** `Yolo_detection Node`
    *   **Subscriber(s):** `Tracking Node`

*   **`/detections_overlay`**
    *   **Message Type:** `sensor_msgs/msg/Image`
    *   **Description:** The raw camera image with detection bounding boxes drawn on it for visualization.
    *   **Publisher(s):** `Yolo_detection Node`
    *   **Subscriber(s):** `UI Node`

*   **`/centroid_status`**
    *   **Message Type:** Custom (e.g., `air_defense_interfaces/msg/CentroidStatus`) containing position (e.g., pixel coordinates or 3D point) and ID of tracked objects. **Needs definition.**
    *   **Description:** Real-time positional information of tracked targets.
    *   **Publisher(s):** `Tracking Node`
    *   **Subscriber(s):** `Command_handler Node`

*   **`/tracked_object`**
    *   **Message Type:** Custom (e.g., `air_defense_interfaces/msg/TrackedObject`) potentially containing detailed state (velocity, history, confidence). **Needs definition.**
    *   **Description:** Comprehensive state information for tracked objects.
    *   **Publisher(s):** `Tracking Node`
    *   **Subscriber(s):** `Command_handler Node`

*   **`/motor/commands`**
    *   **Message Type:** Custom (e.g., `air_defense_interfaces/msg/MotorCommand`) or `geometry_msgs/msg/Twist`. **Needs definition/verification.**
    *   **Description:** Commands for motor control hardware (via `Serial Node`), specifying target angles or velocities.
    *   **Publisher(s):** `Command_handler Node`
    *   **Subscriber(s):** `Serial Node`

*   **`/laser/commands`**
    *   **Message Type:** Custom (e.g., `air_defense_interfaces/msg/LaserCommand`) or `std_msgs/msg/Bool`. **Needs definition/verification.**
    *   **Description:** Commands for the laser mechanism (via `Serial Node`), e.g., fire enable/disable.
    *   **Publisher(s):** `Command_handler Node`
    *   **Subscriber(s):** `Serial Node`

*   **`/motor/feedback`**
    *   **Message Type:** Custom (e.g., `air_defense_interfaces/msg/MotorFeedback`) or `sensor_msgs/msg/JointState`. **Needs definition/verification.**
    *   **Description:** Feedback from motor hardware (via `Serial Node`), e.g., current angles, status.
    *   **Publisher(s):** `Serial Node`
    *   **Subscriber(s):** `System_monitor Node`

*   **`/laser/status`**
    *   **Message Type:** Custom (e.g., `air_defense_interfaces/msg/LaserStatus`) or `std_msgs/msg/Bool`. **Needs definition/verification.**
    *   **Description:** Feedback from laser hardware (via `Serial Node`), indicating state (e.g., ready, firing, error).
    *   **Publisher(s):** `Serial Node`
    *   **Subscriber(s):** `System_monitor Node`

*   **`/system/health`**
    *   **Message Type:** Custom (e.g., `air_defense_interfaces/msg/SystemHealth`) containing status indicators, error flags, diagnostics. **Needs definition.**
    *   **Description:** Consolidated health status of the system.
    *   **Publisher(s):** `System_monitor Node`
    *   **Subscriber(s):** `UI Node`

*   **`/ui/commands`**
    *   **Message Type:** Custom (e.g., `air_defense_interfaces/msg/UICommand`) or `std_msgs/msg/String`. **Needs definition/verification.**
    *   **Description:** High-level commands from the UI (e.g., start/stop, switch mode).
    *   **Publisher(s):** `UI Node`
    *   **Subscriber(s):** `Command_handler Node`

*   **`/system/mode`**
    *   **Message Type:** `std_msgs/msg/String` or `std_msgs/msg/Int8`. **Needs definition/verification.**
    *   **Description:** Current operating mode (e.g., "IDLE", "AUTO_AIM", "MANUAL", "ERROR").
    *   **Publisher(s):** `Command_handler Node`
    *   **Subscriber(s):** `UI Node`

---

## Launch Files

Launch files (`.launch.py`) are Python scripts used to start multiple nodes, configure parameters, and manage the system execution using the `ros2 launch` command.

*   **`main_bringup.launch.py` (Example Placeholder):** The primary launch file to start the entire robot system for competition runs. It should launch all necessary nodes (`camera_node`, `yolo_node`, `tracking_node`, `command_handler_node`, `serial_bridge_node`, `system_monitor_node`, potentially `ui_node`), load parameters from YAML files, and set up remappings if needed.

    ```bash
    # Example Usage (replace with actual launch file name and package)
    ros2 launch main main_bringup.launch.py [launch arguments...]
    ```

---

## Configuration

*   Parameters for nodes should be defined in YAML files (e.g., `config/main_params.yaml`) and loaded via launch files. Key parameters include model paths, thresholds, serial port details, camera settings, etc.
*   Hardware configurations (e.g., serial port, camera details) should always be parameterized for flexibility.