#!/usr/bin/env python3  


import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class VideoSubscriber(Node):
    def __init__(self):
        super().__init__('video_subscriber')
        
        # Create a subscriber to the 'video_frames' topic
        self.subscription = self.create_subscription(
            Image,
            'camera/image_raw',
            self.listener_callback,
            10  # Queue size
        )
        self.subscription  # Prevent unused variable warning
        
        # Initialize CV Bridge to convert ROS Image to OpenCV format
        self.bridge = CvBridge()

    def listener_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Display the image using OpenCV
            cv2.imshow("Video Frame", cv_image)
            cv2.waitKey(1)  # Wait 1ms to allow OpenCV to refresh the window
        except Exception as e:
            self.get_logger().error(f"Failed to process image: {e}")

def main(args=None):
    rclpy.init(args=args)
    
    # Create the subscriber node
    video_subscriber = VideoSubscriber()
    
    try:
        # Spin the node to process incoming messages
        rclpy.spin(video_subscriber)
    except KeyboardInterrupt:
        pass  # Handle Ctrl+C gracefully
    finally:
        # Cleanup
        video_subscriber.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()  # Close OpenCV windows

if __name__ == '__main__':
    main()