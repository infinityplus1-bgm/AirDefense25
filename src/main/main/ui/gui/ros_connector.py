"""
Handles communication between the GUI and ROS2.
This class contains stubs for ROS2 publishers, subscribers, etc.
Replace print statements with actual rclpy calls for ROS2 integration.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
from interfaces.msg import Command, Float32MultiArray2D
from cv_bridge import CvBridge
from PyQt5.QtCore import QObject, pyqtSignal
from PyQt5.QtGui import QImage, QPixmap
import logging
import message_filters
import cv2
import numpy as np
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from main import config as cfg

logger = logging.getLogger(__name__)

class ROSConnector(Node, QObject):
    # Signals for GUI updates based on (simulated) ROS events
    ros_log_received = pyqtSignal(str)
    system_status_updated = pyqtSignal(dict)
    pid_values_received = pyqtSignal(str, float, float, float) # motor_type, p, i, d
    image_received = pyqtSignal(QPixmap)
    target_list_updated = pyqtSignal(list)

    def __init__(self):
        Node.__init__(self, cfg.NODE_UI)
        QObject.__init__(self)




        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT, # Or RMW_QOS_POLICY_RELIABILITY_RELIABLE if message loss is critical
            history=HistoryPolicy.KEEP_LAST,
            depth=20 # Queue depth for messages
        )
        

        self.bridge = CvBridge()
        self.command_publisher = self.create_publisher(Command, cfg.TOPIC_SYSTEM_COMMANDS, 10)
        self.mode_publisher = self.create_publisher(Int32, cfg.TOPIC_SYSTEM_MODE, 10)
        self.target_publisher = self.create_publisher(Int32, cfg.TOPIC_TARGET_ID, 10)
        self.status_publisher = self.create_publisher(Int32, cfg.TOPIC_SYSTEM_STATUS, 10)

        self.qos_profile = qos_profile
        self.view_mode = "Tracking"  # Default view mode
        self.raw_image_sub = None
        self.sync_sub = None
        self.setup_subscriptions()

        logger.info("[ROSConnector] Initialized")


    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            height, width, channel = cv_image.shape
            bytes_per_line = 3 * width
            q_image = QImage(cv_image.data, width, height, bytes_per_line, QImage.Format_RGB888).rgbSwapped()
            pixmap = QPixmap.fromImage(q_image)
            self.image_received.emit(pixmap)
        except Exception as e:
            logger.error(f"Failed to process image: {e}")

    def setup_subscriptions(self):
        """Sets up ROS subscriptions based on the current view mode."""
        # Clean up existing subscriptions
        if self.raw_image_sub:
            self.destroy_subscription(self.raw_image_sub)
            self.raw_image_sub = None
        if self.sync_sub:
            # For message_filters, we can't easily destroy the synchronizer,
            # so we just stop processing its callbacks by setting it to None.
            # The underlying subscriptions will be garbage collected if not referenced elsewhere.
            self.sync_sub = None

        if self.view_mode == "Camera":
            self.raw_image_sub = self.create_subscription(
                Image, cfg.TOPIC_CAMERA_IMAGE_RAW, self.image_callback, self.qos_profile
            )
            logger.info("Subscribed to raw camera view.")
        elif self.view_mode == "Tracking":
            image_sub = message_filters.Subscriber(self, Image, cfg.TOPIC_CAMERA_IMAGE_RAW, qos_profile=self.qos_profile)
            results_sub = message_filters.Subscriber(self, Float32MultiArray2D, cfg.TOPIC_RESULTS, qos_profile=self.qos_profile)
            
            self.sync_sub = message_filters.ApproximateTimeSynchronizer(
                [image_sub, results_sub], queue_size=30, slop=0.1
            )
            self.sync_sub.registerCallback(self.synchronized_callback)
            logger.info("Subscribed to synchronized tracking view.")

    def synchronized_callback(self, image_msg, results_msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
            results = np.array(results_msg.data, dtype=np.float32).reshape(results_msg.rows, results_msg.cols)
            
            target_ids = []
            for result in results:
                x1, y1, x2, y2, obj_id, conf, _ = result
                obj_id = int(obj_id)
                target_ids.append(obj_id)

                # Draw bounding box and ID text
                x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
                color = (0, 255, 0)
                cv2.rectangle(cv_image, (x1, y1), (x2, y2), color, 2)
                text = f"ID: {obj_id}"
                cv2.putText(cv_image, text, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)

            self.target_list_updated.emit(target_ids)

            height, width, _ = cv_image.shape
            bytes_per_line = 3 * width
            q_image = QImage(cv_image.data, width, height, bytes_per_line, QImage.Format_RGB888).rgbSwapped()
            self.image_received.emit(QPixmap.fromImage(q_image))

        except Exception as e:
            logger.error(f"Failed to process synchronized image: {e}")

    def set_view_mode(self, view_mode: str):
        """Switches the view mode between 'Camera' and 'Tracking'."""
        if self.view_mode != view_mode:
            self.view_mode = view_mode
            self.setup_subscriptions()
            logger.info(f"View mode set to: {view_mode}")

    def publish_command(self, command_id: int, values: list):
        """Publishes a command to the /system/commands topic."""
        msg = Command()
        msg.id = command_id
        msg.values = [float(v) for v in values]
        self.command_publisher.publish(msg)
        logger.info(f"Published command: ID={command_id}, Values={msg.values}")

    def publish_mode(self, mode_id: int):
        """Publishes the system mode to the /system/mode topic."""
        msg = Int32()
        msg.data = mode_id
        self.mode_publisher.publish(msg)
        logger.info(f"Published mode: {mode_id}")

    def publish_target_id(self, target_id: int):
        """Publishes the selected target ID to the /target_id topic."""
        msg = Int32()
        msg.data = target_id
        self.target_publisher.publish(msg)
        logger.info(f"Published target ID: {target_id}")

    def publish_status(self, status: int):
        """Publishes the system status to the /system/status topic."""
        msg = Int32()
        msg.data = status
        self.status_publisher.publish(msg)
        logger.info(f"Published system status: {status}")

    def shutdown(self):
        """Cleans up ROS2 resources."""
        self.destroy_node()
        logger.info("[ROSConnector] Shutdown")
