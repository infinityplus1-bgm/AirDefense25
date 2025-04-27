import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
from detection.msg import BoundingBox, BoundingBoxes
from detection import Detector

class YoloDetection(Node):
    def __init__(self, model):
        super().__init__('yolo_detection')

        self.detector = Detector(model)
        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image,'/camera/image_raw',self.image_callback,10
        )

        self.detections_publisher = self.create_publisher(BoundingBoxes, '/detections', 10)
        self.overlay_publisher = self.create_publisher(Image, '/detections/overlay', 10)

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        detections = self.detector.detect_objects(frame)

        boxes_msg = BoundingBoxes()

        for det in detections:
            x1, y1, x2, y2, conf = map(float, det[:5])

            cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)

            box = BoundingBox()
            box.x1 = x1
            box.y1 = y1
            box.x2 = x2
            box.y2 = y2
            box.confidence = conf
            boxes_msg.boxes.append(box)

        self.detections_publisher.publish(boxes_msg)
        self.overlay_publisher.publish(self.bridge.cv2_to_imgmsg(frame, encoding='bgr8'))

def main(args=None):
   
    rclpy.init(args=args)
    model = YOLO("best.pt")
    node = YoloDetection(model)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


