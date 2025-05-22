#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
import cv_bridge
import argparse  # Import argparse
import sys       # Keep sys for sys.exit and argument handling

class CameraNode(Node):

    # 1. Modify constructor to accept arguments
    def __init__(self, video_path_arg, timer_period_arg):
        super().__init__('camera')
        # Store arguments if needed
        self.video_path = video_path_arg
        self.timer_period = timer_period_arg

        self.get_logger().info(f"Initializing publisher.")
        self.get_logger().info(f"  Video Source: '{self.video_path}'")
        self.get_logger().info(f"  Publish Period: {self.timer_period} seconds")

        self._publisher = self.create_publisher(Image, 'camera/image_raw', 10)

        # 2. Use the passed timer_period argument
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        # 3. Use the passed video_path argument for VideoCapture
        try:
            if self.video_path == "arducam":
                pipeline = f"v4l2src device=/dev/video4 ! image/jpeg,width=1600,height=1200,framerate=30/1 ! jpegdec ! videoconvert ! appsink"
                
                self.cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
            else:

                # Check if the input path is just digits (likely a webcam index)
                if self.video_path.isdigit():
                    capture_source = int(self.video_path)
                    self.get_logger().info(f"Attempting to use webcam index: {capture_source}")
                else:
                    capture_source = self.video_path
                    self.get_logger().info(f"Attempting to use video file: {capture_source}")

                self.cap = cv2.VideoCapture(capture_source)


            if not self.cap.isOpened():
                # Log error and cleanly exit if capture device fails
                raise IOError(f"Cannot open video source: {capture_source}")

        except Exception as e:
            self.get_logger().error(f"Failed to initialize video capture: {e}")
            # Exit cleanly if we can't open the video source
            # You might want more sophisticated error handling depending on your needs
            sys.exit(f"Critical Error: Could not open video source '{self.video_path}'. Exiting.")

        self.bridge = cv_bridge.CvBridge()
        self.frame_count = 0
        self.get_logger().info('Video source opened successfully. Starting publishing...')

    def timer_callback(self):
        ret, frame = self.cap.read()

        if ret:
            try:
                ros_image = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                # Add timestamp (important!)
                ros_image.header.stamp = self.get_clock().now().to_msg()
                # Add a frame_id if known (replace 'camera_link' as appropriate)
                ros_image.header.frame_id = "camera_link"

                self._publisher.publish(ros_image)
                self.frame_count += 1
                # Log less frequently to avoid console spam
                if self.frame_count % 100 == 0: # e.g., log every 100 frames
                     self.get_logger().info(f'Publishing image frame {self.frame_count}')
            except cv_bridge.CvBridgeError as e:
                 self.get_logger().error(f"CV Bridge error: {e}")
            except Exception as e:
                self.get_logger().error(f"Error during frame publishing: {e}")
        else:
            self.get_logger().warn('Failed to retrieve frame - end of video or camera error?')
            # Consider what to do here: loop video, stop node, etc.
            # For now, we'll just keep trying (which might spin CPU if video ended)
            # A better approach for files might be to stop/destroy the node or loop.

    def destroy_node(self):
        # Release the capture device cleanly
        self.get_logger().info("Releasing video capture device.")
        if hasattr(self, 'cap') and self.cap.isOpened():
            self.cap.release()
        super().destroy_node()

def main(cli_args=None): 

        # TODO : ADD ZOOM CAPABILITY
    if cli_args is None:
       cli_args = sys.argv[1:] # Exclude script name from args passed to parser

    # parse the arguments
    parser = argparse.ArgumentParser(
        description="ROS 2 Node to publish images from video or webcam."
    )
    parser.add_argument(
        "-v", "--video-path",
        type=str,
        required=True,
        help="REQUIRED: Path to video file or webcam index (e.g., '0', '1')."
    )
    parser.add_argument(
        "-t", "--timer-period",
        type=float,
        default=0.08, # Default ~12.5 FPS
        help="Timer period in seconds for publishing images (default: 0.08)"
    )

    app_args, ros_args = parser.parse_known_args(cli_args) # this will parse the argument for the program and return the other ones for ROS

    # use the returned arguments
    rclpy.init(args=ros_args)

    try:
        # create an object from the node and pass the arguments to the constructor
        camera_publisher = CameraNode(
            video_path_arg=app_args.video_path,
            timer_period_arg=app_args.timer_period
        )
        # Spin the node to keep it alive and executing callbacks
        rclpy.spin(camera_publisher)

    except KeyboardInterrupt:
        print("Keyboard interrupt received, shutting down.")
    except Exception as e:
        # Log any issues
        temp_logger = rclpy.logging.get_logger("main_exception_logger")
        temp_logger.error(f"Unhandled exception in main: {e}")
    finally:
        # this will run whether the user interrupted or any other exception happened
        # it will destoy the node properly
        if 'camera_publisher' in locals() and isinstance(camera_publisher, Node) and rclpy.ok():
            camera_publisher.destroy_node()
        if rclpy.ok():
             rclpy.shutdown()

if __name__ == "__main__":
    main()