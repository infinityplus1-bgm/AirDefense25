import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

from interfaces.msg import Float32MultiArray2D
import message_filters
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import cv2
import cv_bridge
import numpy as np

class image_processing_test_node(Node):
    def __init__(self):
        super().__init__('tracker_test')


        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT, # Or RMW_QOS_POLICY_RELIABILITY_RELIABLE if message loss is critical
            history=HistoryPolicy.KEEP_LAST,
            depth=20 # Queue depth for messages
        )

        self.image_subscription = message_filters.Subscriber(self , Image , '/camera/image_raw' , qos_profile=qos_profile)
        self.tracked_objects_subscription = message_filters.Subscriber(self , Float32MultiArray2D , '/results' , qos_profile=qos_profile)

        self.synchronizer = message_filters.ApproximateTimeSynchronizer(
            [self.image_subscription , self.tracked_objects_subscription],
            queue_size=30,
            slop=0.1,
            allow_headerless=False
        )

        
        self.synchronizer.registerCallback(
            self.display_callback
        )

        self.bridge = cv_bridge.CvBridge()
        # self.current_frame = None
        # self.current_tracked_objects = None
        # self.display_timer = self.create_timer(0.033, self.display_callback) # ~30 FPS for display

        self.get_logger().info("Waiting for image and tracked objects data...")


    def display_callback(self , image : Image , tracked_objects : Float32MultiArray2D):
        # convert the image to opencv 
        frame = self.bridge.imgmsg_to_cv2(image, desired_encoding='bgr8')
        # convert the results to numpy array 
        results : np.ndarray = np.array(tracked_objects.data , dtype=np.float32).reshape(tracked_objects.rows , tracked_objects.cols)

        print(results)
        print(results.shape)
        # since the results are flattened we get them back in shape

        for result in results:
            x1, y1, x2, y2, obj_id , conf , _ = result
            x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
            obj_id = int(obj_id)

            # Draw bounding box
            color = (0, 255, 0)  # Green color for bounding box
            thickness = 2
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, thickness)

            # Put ID text
            text = f"ID: {obj_id}"
            font = cv2.FONT_HERSHEY_SIMPLEX
            font_scale = 0.7
            font_thickness = 2
            text_size = cv2.getTextSize(text, font, font_scale, font_thickness)[0]
            
            # Position the text above the bounding box
            text_x = x1
            text_y = y1 - 10 if y1 - 10 > text_size[1] else y1 + text_size[1] + 5 # Ensure text is visible

            cv2.putText(frame, text, (text_x, text_y), font, font_scale, color, font_thickness, cv2.LINE_AA)

        # Display the frame
        cv2.imshow("Tracked Objects Display", frame)
        cv2.waitKey(1) # Refresh window, 1ms delay

        
    def destroy_node(self):
        """Cleanly destroy the node and close OpenCV windows."""
        self.get_logger().info("Shutting down display node. Closing OpenCV windows.")
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    display_node = image_processing_test_node()
    try:
        rclpy.spin(display_node)
    except KeyboardInterrupt:
        display_node.get_logger().info('Keyboard Interrupt received, shutting down.')
    finally:
        display_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()