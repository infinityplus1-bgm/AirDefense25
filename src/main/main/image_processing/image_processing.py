from ultralytics import YOLO
import numpy as np
import cv2

from main.config import IMAGE_PROCESSING_MODEL_PATH
from ultralytics.engine.results import Results
from torch import Tensor

class image_processing:
    def __init__(self):
        
        self.model = YOLO(IMAGE_PROCESSING_MODEL_PATH)

        
    def process(self , frame : np.ndarray , color_detection = False) -> Tensor:
        # if the color detection is turned on then detect colours first
        if color_detection:
            frame = self.color_detector(frame)
        # update the tracker
        results : Results = self.model.track(source = frame , persist=True)

        # organize the detected ob  ject by size and return them in a useful way
        return results[0].boxes.data


    
    def color_detector(self , frame : np.ndarray):

        hsv_image = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)

        # 2- define the range of red
        # lower=np.array([-10, 100, 100])
        # upper=np.array([10, 255, 255])

        lower_red1 = np.array([0, 100, 100])
        upper_red1 = np.array([10, 255, 255])

        lower_red2 = np.array([160, 100, 100]) # Adjust as needed
        upper_red2 = np.array([179, 255, 255])

        # Create masks for each red range
        mask1 = cv2.inRange(hsv_image, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv_image, lower_red2, upper_red2)

        # Combine the masks
        full_red_mask = cv2.bitwise_or(mask1, mask2)


        # Red_mask = cv2.inRange(HSV,lower, upper)
        result = cv2.bitwise_and(frame, frame, mask = full_red_mask)

        return result

