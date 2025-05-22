import rclpy
from rclpy.node import Node
import numpy as np
from main.detection.msg import BoundingBox, BoundingBoxes
from main.tracking.msg import TrackedObject, TrackedObjectArray, CentroidStatus, CentroidStatusArray
from std_msgs.msg import Int32

class ObjectTracker(Node):
    def __init__(self):
        super().__init__('object_tracker')
        
        
        self.detection_sub = self.create_subscription( BoundingBoxes,'/detections',self.detection_callback,10)
        
        self.centroid_publisher = self.create_publisher(CentroidStatusArray,'/centroid_status',10)
        
        self.tracked_object_publisher = self.create_publisher(TrackedObjectArray,'/tracked_object',10 )
        
        self.get_logger().info('Object Tracker node initialized')
    
    def detection_callback(self, msg):
        """Process incoming detections and update tracking"""
        # Convert detection message to numpy array for tracker
        if not msg.boxes:
            # No detections, publish empty results
            tracked_boxes = TrackedObjectArray()
            self.tracked_object_publisher.publish(tracked_boxes)
            return
            
        detections = np.zeros((len(msg.boxes), 5))
        for i, box in enumerate(msg.boxes):
            detections[i] = [box.x1, box.y1, box.x2, box.y2, box.confidence]
        
        # Update tracker with new detections
        tracked_objects = self.tracker.update(detections)
        
        # Create message for tracked objects
        tracked_boxes = BoundingBoxes()
        
        # Process each tracked object
        for obj in tracked_objects:
            x1, y1, x2, y2, track_id = obj
            
            # Create tracked box message with track_id
            tracked_box = BoundingBox()
            tracked_box.x1 = float(x1)
            tracked_box.y1 = float(y1)
            tracked_box.x2 = float(x2)
            tracked_box.y2 = float(y2)
            tracked_box.confidence = 1.0  # Tracking result doesn't have confidence
            tracked_box.track_id = int(track_id)
            
            tracked_boxes.boxes.append(tracked_box)
            
            # Calculate centroid
            xa = int((x1 + x2) / 2)
            ya = int((y1 + y2) / 2)
            
            centroid_msg = Point()
            centroid_msg.x = float(xa)
            centroid_msg.y = float(ya)
            centroid_msg.z = float(track_id)  # Using z to store track_id
            
            self.centroid_publisher.publish(centroid_msg)
        
        # Publish tracked boxes
        self.tracked_object_publisher.publish(tracked_boxes)

def main(args=None):
    rclpy.init(args=args)
    node = ObjectTracker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()