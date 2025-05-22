import rclpy
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from pprint import pprint
from main.tracking.tracker import Tracker 
from main.tracking.sort import *




class TrackingNode(Node):
    def __init__(self , tracker):
        super().__init__('tracking')

        self.tracker = Tracker(tracker)

        self.subscription = self.create_subscription(
            Float32MultiArray,'detections',self.detection_callback,10
        )



        self.tracked_objects_publisher = self.create_publisher(Float32MultiArray, 'tracked_objects', 10)
        # self.detections_publisher = self.create_publisher(Float32MultiArray, '', 10)


    def detection_callback(self , flattened_detections : Float32MultiArray):
        # since it was flattened to a 2d array to be published we reconstruct the matrix
        
        # first convert the ros message to a numpy array
        flattened_detections : np.ndarray = np.array(flattened_detections.data , dtype=np.float32)

        # now we reshape to get a 2d matrix

        # get the number of flattened_detections
        detections_count = flattened_detections.shape[0] // 5

        self.get_logger().info(f"received {detections_count} detection")
        detections : np.ndarray = flattened_detections.reshape((detections_count , 5)) # 5 is x1 , y1 , x2 , y2 , conf

        tracked_objects : np.ndarray = self.tracker.track_objects(detections)

        # flatten the results to be able to send
        # results will be x1 , y1 , x2 , y2 , id
        tracked_objects_msg : Float32MultiArray = Float32MultiArray()

        tracked_objects_msg.data = [float(item) for target in tracked_objects for item in target]

        self.tracked_objects_publisher.publish(tracked_objects_msg)

        


def main(args=None):
   
    rclpy.init(args=args)
    tracker = Sort(max_age=120, min_hits=3, iou_threshold=0.3)
    node = TrackingNode(tracker= tracker)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()