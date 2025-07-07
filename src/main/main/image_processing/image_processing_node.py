import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32, String
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
from ultralytics.engine.results import Results
from main.image_processing.image_processing import image_processing
from pprint import pprint
from interfaces.msg import Float32MultiArray2D
from torch import Tensor
import logging
from main.utils.logging_config import setup_logging
from main import config

logger = logging.getLogger(__name__)

class image_processing_node(Node):
    def __init__(self):
        super().__init__(config.NODE_IMAGE_PROCESSING)

        # self.processor = Detector(model)
        self.processor = image_processing()
        self.system_enabled = False

        self.bridge = CvBridge()

        self.status_subscriber = self.create_subscription(
            Int32,
            config.TOPIC_SYSTEM_STATUS,
            self.system_status_callback,
            10)

        self.subscription = self.create_subscription(
            Image,config.TOPIC_CAMERA_IMAGE_RAW,self.image_callback,10
        )
        # since ros doesn't support multi dimensional array messages we will flatten all arrays into 1d and then send it
        #  and the receiver will recreate the results
        self.publisher = self.create_publisher(Float32MultiArray2D, config.TOPIC_RESULTS, 10)
        self.health_publisher = self.create_publisher(String, config.TOPIC_HEALTH_IMAGE_PROCESSING_NODE, 10)
        self.health_timer = self.create_timer(1, self.publish_health_status)

    def publish_health_status(self):
        msg = String()
        msg.data = 'healthy'
        self.health_publisher.publish(msg)

    def system_status_callback(self, msg):
        self.system_enabled = bool(msg.data)
        if self.system_enabled:
            logger.info('System enabled. Image processing node is active.')
        else:
            logger.info('System disabled. Image processing node is inactive.')
        
    def image_callback(self, image : Image):
        if not self.system_enabled:
            return
        frame = self.bridge.imgmsg_to_cv2(image, desired_encoding='bgr8')

        results  = self.processor.process(frame , color_detection=False)

        # flatten and create the publish message
        msg = Float32MultiArray2D()
        # copy the timestamp for synchronization
        msg.header.stamp = image.header.stamp

        # flatten the results and add
        msg.cols = results.shape[1]
        msg.rows = results.shape[0]
        # print(results)
        msg.data = results.flatten().tolist()

        self.publisher.publish(msg)
        logger.debug(f"Published results with shape ({msg.rows}, {msg.cols})")
def main(args=None):
    setup_logging()
    rclpy.init(args=args)
    node = image_processing_node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
