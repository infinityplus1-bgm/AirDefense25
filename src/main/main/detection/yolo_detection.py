import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32, String
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
from ..config import YOLO_MODEL_PATH
# from std_msgs.msg import Float32MultiArray
from main.detection.detection import Detector
from pprint import pprint
from interfaces.msg import Float32MultiArray2D , Float32MultiArray
import numpy as np

def np2ros_float(src : np.ndarray):
    result = []
    for row in src:
        tmp : Float32MultiArray = Float32MultiArray()
        tmp.row_data = row
        result.append(Float32MultiArray)
    return result


class YoloDetection(Node):
    def __init__(self, model):
        super().__init__('yolo_detection')

        self.detector = Detector(model)
        self.bridge = CvBridge()
        self.system_enabled = False

        self.status_subscriber = self.create_subscription(
            Int32,
            '/system/status',
            self.system_status_callback,
            10)

        self.subscription = self.create_subscription(
            Image,'camera/image_raw',self.image_callback,10
        )
        # since ros doesn't support multi dimensional array messages we will flatten all arrays into 1d and then send it
        #  and the receiver will recreate the results
        self.detections_publisher = self.create_publisher(Float32MultiArray2D, '/detections', 10)
        self.overlay_publisher = self.create_publisher(Image, '/detections/overlay', 10)
        self.health_publisher = self.create_publisher(String, '/health/detection_node', 10)
        self.health_timer = self.create_timer(1, self.publish_health_status)

    def publish_health_status(self):
        msg = String()
        msg.data = 'healthy'
        self.health_publisher.publish(msg)

    def system_status_callback(self, msg):
        self.system_enabled = bool(msg.data)
        if self.system_enabled:
            self.get_logger().info('System enabled. Detection node is active.')
        else:
            self.get_logger().info('System disabled. Detection node is inactive.')

    def image_callback(self, msg : Image):
        if not self.system_enabled:
            return
            
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        detections = self.detector.detect_objects(frame)

        # overlay detections on top of frame
        for det in detections:
            x1, y1, x2, y2, conf = map(float, det[:5])
            cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)

        
        boxes_msg = Float32MultiArray2D()
        # set the shape of the data
        boxes_msg.rows = detections.shape[0]
        boxes_msg.cols = detections.shape[1]
        # pprint(detections)
        boxes_msg.data = np2ros_float(detections)
        # flatted the 2d array into 1d to be able to send
        # boxes_msg.data = [float(item) for detection in detections for item in detection]
        

        # we have to do this to synchronize frames with results in the UI
        boxes_msg.header.stamp = msg.header.stamp

        self.detections_publisher.publish(boxes_msg)
        self.overlay_publisher.publish(self.bridge.cv2_to_imgmsg(frame, encoding='bgr8'))


def main(args=None):
    rclpy.init(args=args)
    model = YOLO(YOLO_MODEL_PATH)
    node = YoloDetection(model)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
