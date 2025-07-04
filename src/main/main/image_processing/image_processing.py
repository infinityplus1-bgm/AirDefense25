from ultralytics import YOLO
import numpy as np
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
        pass
