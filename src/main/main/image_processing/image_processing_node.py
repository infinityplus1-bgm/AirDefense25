import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
from ultralytics.engine.results import Results
# from std_msgs.msg import Float32MultiArray
# from main.detection.detection import Detector
from main.image_processing.image_processing import image_processing
from pprint import pprint
from interfaces.msg import Float32MultiArray2D
from torch import Tensor



class image_processing_node(Node):
    def __init__(self):
        super().__init__('image_processing')

        # self.processor = Detector(model)
        self.processor = image_processing()

        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image,'camera/image_raw',self.image_callback,10
        )
        # since ros doesn't support multi dimensional array messages we will flatten all arrays into 1d and then send it
        #  and the receiver will recreate the results
        self.publisher = self.create_publisher(Float32MultiArray2D, '/results', 10)
        
    def image_callback(self, image : Image):
        frame = self.bridge.imgmsg_to_cv2(image, desired_encoding='bgr8')

        results  = self.processor.process(frame , color_detection=False)

        # flatten and create the publish message
        msg = Float32MultiArray2D()
        # copy the timestamp for synchronization
        msg.header.stamp = image.header.stamp

        # flatten the results and add
        msg.cols = results.shape[1]
        msg.rows = results.shape[0]
        # print(results)
        msg.data = results.flatten().tolist()

        self.publisher.publish(msg)
        # # Draw bounding box
        # color = (0, 255, 0)  # Green color for bounding box
        # thickness = 2

        # font_scale = 0.7
        # font_thickness = 2
        # # text_size = cv2.getTextSize(text, font, font_scale, font_thickness)[0]
        # font = cv2.FONT_HERSHEY_SIMPLEX

        # # for result in results['big']:
        # #     x1 , y1 , x2 , y2 , obj_id , conf , class_id = results.data
        # #     text = f"ID: {obj_id} | BIG"
        # #     text_x = x1
        # #     text_y = y1 - 10
        # #     cv2.rectangle(frame, (x1, y1), (x2, y2), color, thickness)
        # #     cv2.putText(display_frame, text, (text_x, text_y), font, font_scale, color, font_thickness, cv2.LINE_AA)    

        # # for result in results['small']

        
        
        # # Position the text above the bounding box
        
        #  if y1 - 10 > text_size[1] else y1 + text_size[1] + 5 # Ensure text is visible

        

        
        

        # # overlay detections on top of frame
        # for det in detections:
        #     x1, y1, x2, y2, conf = map(float, det[:5])
        #     cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)

        
        # boxes_msg = Float32MultiArray2D()
        # # set the shape of the data
        # boxes_msg.rows = detections.shape[0]
        # boxes_msg.cols = detections.shape[1]
        # # pprint(detections)
        # boxes_msg.data = np2ros_float(detections)
        # # flatted the 2d array into 1d to be able to send
        # # boxes_msg.data = [float(item) for detection in detections for item in detection]
        

        # # we have to do this to synchronize frames with results in the UI
        # boxes_msg.header.stamp = msg.header.stamp

        # self.detections_publisher.publish(boxes_msg)
        # self.overlay_publisher.publish(self.bridge.cv2_to_imgmsg(frame, encoding='bgr8'))

def main(args=None):
   
    rclpy.init(args=args)
    node = image_processing_node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


