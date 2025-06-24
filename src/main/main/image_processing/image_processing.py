from ultralytics import YOLO
import numpy as np
from typing import Dict , Tuple , List
from ultralytics.engine.results import Results
from torch import Tensor , tensor , float32
import torch
class image_processing:
    def __init__(self):
        
        self.model = YOLO("/home/infinityplusone/Documents/AirDefense25/assets/best.pt")

        
    def process(self , frame : np.ndarray , color_detection = False) -> Tensor:
        # if the color detection is turned on then detect colours first
        if color_detection:
            frame = self.color_detector(frame)
        # update the tracker
        results : Results = self.model.track(source = frame , persist=True)
        # print(results[0].boxes)

        
        # print()
        # objects = torch.cat([
        #             box.xyxy
        #             for detection in results 
        #             for box in detection.boxes
        #         ])
        # organize the detected ob  ject by size and return them in a useful way
        return results[0].boxes.data


    
    def color_detector(self , frame : np.ndarray):
        pass

