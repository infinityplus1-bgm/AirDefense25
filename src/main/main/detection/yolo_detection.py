import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
from std_msgs.msg import Float32MultiArray
from main.detection.detection import Detector
from pprint import pprint


class YoloDetection(Node):
    def __init__(self, model):
        super().__init__('yolo_detection')

        self.detector = Detector(model)
        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image,'camera/image_raw',self.image_callback,10
        )
        # since ros doesn't support multi dimensional array messages we will flatten all arrays into 1d and then send it
        #  and the receiver will recreate the results
        self.detections_publisher = self.create_publisher(Float32MultiArray, '/detections', 10)
        self.overlay_publisher = self.create_publisher(Image, '/detections/overlay', 10)

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        detections = self.detector.detect_objects(frame)

        
        

        # overlay detections on top of frame
        for det in detections:
            x1, y1, x2, y2, conf = map(float, det[:5])
            cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)

        
        boxes_msg = Float32MultiArray()
        # flatted the 2d array into 1d to be able to send
        boxes_msg.data = [float(item) for detection in detections for item in detection]



        self.detections_publisher.publish(boxes_msg)
        self.overlay_publisher.publish(self.bridge.cv2_to_imgmsg(frame, encoding='bgr8'))

def main(args=None):
   
    rclpy.init(args=args)
    model = YOLO("best.pt")
    node = YoloDetection(model)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


