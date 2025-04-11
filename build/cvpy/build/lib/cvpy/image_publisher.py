import rclpy

from sensor_msgs.msg import Image

import sys

from rclpy.node import Node

import cv2

import cv_bridge


class ImagePublisher(Node):
    def __init__(self):
        super().__init__('ImagePublisher')
        self.publisher_= self.create_publisher(Image, 'image_data',10)
        timer_period= float(sys.argv[1]) if len(sys.argv) > 1 else 0.08
        self.timer = self.create_timer(timer_period,self.timer_callback)
        self.cap = cv2.VideoCapture('/home/user/sample.mp4') # change to your path or to 0 for webcam
        self.bridge = cv_bridge.CvBridge()


    def timer_callback(self):
        ret, frame = self.cap.read()

        if ret:
            ros_image = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.publisher_.publish(ros_image)
            self.get_logger().info('Publishing image frame')

    def destroy_node(self):
        self.cap.release()
        super().destroy_node() 

def main(args=None):
    rclpy.init(args=args)
    camera_publisher = ImagePublisher()
    rclpy.spin(camera_publisher)
    camera_publisher.destroy_node()
    rclpy.shutdown()

if __name__=="__main__":
    main()