#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import message_filters
import cv2
import cv_bridge
import numpy as np

# class TrackerTestNode(Node):
#     def __init__(self):
#         super().__init__('tracker_test')

#         # Subscribe to the camera image topic
#         self.image_subscription = self.create_subscription(
#             Image,
#             'camera/image_raw',
#             self.image_callback,
#             10
#         )

#         # Subscribe to the tracked objects topic
#         self.tracked_objects_subscription = self.create_subscription(
#             Float32MultiArray,
#             'tracked_objects',
#             self.tracked_objects_callback,
#             10
#         )

#         self.bridge = cv_bridge.CvBridge()
#         self.current_frame = None
#         self.current_tracked_objects = None
#         self.display_timer = self.create_timer(0.033, self.display_callback) # ~30 FPS for display

#         self.get_logger().info("Waiting for image and tracked objects data...")

#     def image_callback(self, msg: Image):
#         """Callback for receiving camera images."""
#         try:
#             # Convert ROS Image message to OpenCV image
#             self.current_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
#             # self.get_logger().debug(f"Received image with timestamp: {msg.header.stamp}")
#         except cv_bridge.CvBridgeError as e:
#             self.get_logger().error(f"CvBridge Error in image_callback: {e}")
#             self.current_frame = None # Clear frame on error

#     def tracked_objects_callback(self, msg: Float32MultiArray):
#         """Callback for receiving tracked objects data."""
#         # Convert the flattened list back to a NumPy array
#         # Each tracked object is (x1, y1, x2, y2, id)
#         data = np.array(msg.data, dtype=np.float32)

#         # Check if the data is empty or malformed
#         if data.size == 0:
#             self.current_tracked_objects = np.array([])
#             return
        
#         # Ensure the array can be reshaped into (N, 5)
#         if data.size % 5 != 0:
#             self.get_logger().warn(f"Received malformed tracked_objects data. Size: {data.size}, not divisible by 5. Ignoring this message.")
#             self.current_tracked_objects = np.array([])
#             return

#         self.current_tracked_objects = data.reshape((-1, 5))
#         # self.get_logger().debug(f"Received {self.current_tracked_objects.shape[0]} tracked objects.")


#     def display_callback(self):
#         """Timer callback to draw and display the image."""
#         if self.current_frame is not None:
#             display_frame = self.current_frame.copy() # Work on a copy

#             if self.current_tracked_objects is not None and self.current_tracked_objects.size > 0:
                # for obj in self.current_tracked_objects:
                #     x1, y1, x2, y2, obj_id = obj
                #     x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
                #     obj_id = int(obj_id)

                #     # Draw bounding box
                #     color = (0, 255, 0)  # Green color for bounding box
                #     thickness = 2
                #     cv2.rectangle(display_frame, (x1, y1), (x2, y2), color, thickness)

                #     # Put ID text
                #     text = f"ID: {obj_id}"
                #     font = cv2.FONT_HERSHEY_SIMPLEX
                #     font_scale = 0.7
                #     font_thickness = 2
                #     text_size = cv2.getTextSize(text, font, font_scale, font_thickness)[0]
                    
                #     # Position the text above the bounding box
                #     text_x = x1
                #     text_y = y1 - 10 if y1 - 10 > text_size[1] else y1 + text_size[1] + 5 # Ensure text is visible

                #     cv2.putText(display_frame, text, (text_x, text_y), font, font_scale, color, font_thickness, cv2.LINE_AA)

#             # Display the frame
            # cv2.imshow("Tracked Objects Display", display_frame)
            # cv2.waitKey(1) # Refresh window, 1ms delay

#     def destroy_node(self):
#         """Cleanly destroy the node and close OpenCV windows."""
#         self.get_logger().info("Shutting down display node. Closing OpenCV windows.")
#         cv2.destroyAllWindows()
#         super().destroy_node()


class TrackerTestNode(Node):
    def __init__(self):
        super().__init__('tracker_test')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT, # Or RMW_QOS_POLICY_RELIABILITY_RELIABLE if message loss is critical
            history=HistoryPolicy.KEEP_LAST,
            depth=10 # Queue depth for messages
        )

        # Subscribe to the camera image topic
        self.image_subscription = message_filters.Subscriber(self , Image , 'camera/image_raw' , qos_profile=qos_profile)
        self.tracked_objects_subscription = message_filters.Subscriber(self, Float32MultiArray , 'tracked_objects' , qos_profile=qos_profile)

        self.synchronizer = message_filters.ApproximateTimeSynchronizer(
            [self.image_subscription , self.tracked_objects_subscription],
            queue_size=30,
            slop=0.2,
            allow_headerless=False
        )
        # # Subscribe to the camera image topic
        # self.image_subscription = self.create_subscription(
        #     Image,
        #     'camera/image_raw',
        #     self.image_callback,
        #     10
        # )

        # # Subscribe to the tracked objects topic
        # self.tracked_objects_subscription = self.create_subscription(
        #     Float32MultiArray,
        #     'tracked_objects',
        #     self.tracked_objects_callback,
        #     10
        # )

        self.synchronizer.registerCallback(self.tracker_callback)

        self.bridge = cv_bridge.CvBridge()

        self.get_logger().info("Waiting for image and tracked objects data...")


    def tracker_callback(self , image : Image , tracker_results : Float32MultiArray):

        self.get_logger().info(
            f"Received synchronized set: "
            f"Image stamp: {image.header.stamp.sec}.{image.header.stamp.nanosec}, "
            f"Tracking stamp: {tracker_results.header.stamp.sec}.{tracker_results.header.stamp.nanosec}"
        )

        frame = self.bridge.imgmsg_to_cv2(image, desired_encoding='bgr8')

        tracker_results = np.array(tracker_results.data, dtype=np.float32)

        tracker_results = tracker_results.reshape((-1, 5))

        for obj in tracker_results:
                    x1, y1, x2, y2, obj_id = obj
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

        cv2.imshow("Tracked Objects Display", frame)
        cv2.waitKey(1) # Refresh window, 1ms delay


def main(args=None):
    rclpy.init(args=args)
    display_node = TrackerTestNode()
    try:
        rclpy.spin(display_node)
    except KeyboardInterrupt:
        display_node.get_logger().info('Keyboard Interrupt received, shutting down.')
    finally:
        display_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()