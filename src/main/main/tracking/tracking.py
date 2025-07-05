import rclpy
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Int32, String
# from std_msgs.msg import Float32MultiArray
from pprint import pprint
from main.tracking.tracker import Tracker
from main.tracking.sort import *
from main import config as cfg

from interfaces.msg import Float32MultiArray2D , Float32MultiArray

def np2ros_float(src : np.ndarray):
    result = []
    for row in src:
        tmp : Float32MultiArray = Float32MultiArray()
        tmp.row_data = row
        result.append(Float32MultiArray)
    return result


class TrackingNode(Node):
    def __init__(self , tracker):
        super().__init__(cfg.NODE_TRACKING)

        self.tracker = Tracker(tracker)
        self.system_enabled = False

        self.status_subscriber = self.create_subscription(
            Int32,
            cfg.TOPIC_SYSTEM_STATUS,
            self.system_status_callback,
            10)

        self.subscription = self.create_subscription(
            Float32MultiArray2D,cfg.TOPIC_DETECTIONS,self.detection_callback,10
        )



        self.tracked_objects_publisher = self.create_publisher(Float32MultiArray2D, cfg.TOPIC_TRACKED_OBJECTS, 10)
        # self.detections_publisher = self.create_publisher(Float32MultiArray2D, '', 10)
        self.health_publisher = self.create_publisher(String, cfg.TOPIC_HEALTH_TRACKING_NODE, 10)
        self.health_timer = self.create_timer(1, self.publish_health_status)


    def publish_health_status(self):
        msg = String()
        msg.data = 'healthy'
        self.health_publisher.publish(msg)

    def system_status_callback(self, msg):
        self.system_enabled = bool(msg.data)
        if self.system_enabled:
            self.get_logger().info('System enabled. Tracking node is active.')
        else:
            self.get_logger().info('System disabled. Tracking node is inactive.')

    def detection_callback(self , msg : Float32MultiArray2D):
        if not self.system_enabled:
            return
        # since it was flattened to a 2d array to be published we reconstruct the matrix
        
        # first convert the ros message to a numpy array
        detections : np.ndarray = np.array(msg.data , dtype=np.float32)

        # now we reshape to get a 2d matrix

        # get the number of flattened_detections
        detections_count = msg.rows

        self.get_logger().info(f"received {detections_count} detection")
        # detections : np.ndarray = flattened_detections.reshape((detections_count , 5)) # 5 is x1 , y1 , x2 , y2 , conf

        tracked_objects : np.ndarray = self.tracker.track_objects(detections)

        # flatten the results to be able to send
        # results will be x1 , y1 , x2 , y2 , id
        tracked_objects_msg : Float32MultiArray2D = Float32MultiArray2D()

        # set the shape of the data
        tracked_objects_msg.rows = detections.shape[0]
        tracked_objects_msg.cols = detections.shape[1]

        # tracked_objects_msg.data = [
        #     [float(value) for value in row]
        #     for row in tracked_objects
        # ]

        tracked_objects_msg.data = np2ros_float(tracked_objects)
        # we have to do this to synchronize frames with results in the UI
        tracked_objects_msg.header.stamp = msg.header.stamp
        
        
        # tracked_objects_msg.data = [float(item) for target in tracked_objects for item in target]

        self.tracked_objects_publisher.publish(tracked_objects_msg)

        


def main(args=None):
   
    rclpy.init(args=args)
    tracker = Sort(max_age=120, min_hits=3, iou_threshold=0.3)
    node = TrackingNode(tracker= tracker)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
