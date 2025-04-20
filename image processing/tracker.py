from ultralytics import YOLO
import cv2 as cv
import numpy as np

class tracker:
    def __init__(self, model, video_source, tracker, weights):
        """Initialize YOLO model, video source, and tracker."""
        self.raw_model = model
        self.model = model(weights) # TODO:  i don't think the model needs weight check how SORT what does before by @abd eid and implement
        # TODO:  change this because this takes ready detection from topic
        self.video = video_source 
        self.video_capture = cv.VideoCapture(video_source)
        if not self.video_capture.isOpened():
            print("Error: Video file not opened.")
            return
        

        self.mot_tracker = tracker
        
    def track_objects(self, frame):
        #TODO: instead of taking a full frame you will be getting the detection in the format (x1,y1,x2,y2,conf) so change accordingly
        """Track objects using the SORT tracker per frame."""
        results = self.model.track(frame, persist=True, tracker=self.mot_tracker, conf=0.53, iou=0.3, agnostic_nms=True, classes=[0])
        return results
        
    def process_frame(self, frame):
        tracker_results = self.track_objects(frame)

        # TODO : we don't need to write to the frame as it only published the tracker results and the closest distance to the kill zone
        for r in tracker_results:
            for box in r.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                track_id = int(box.id[0])
                conf = float(box.conf[0])

                cv.putText(frame, f'ID {int(track_id)}', (int(x1), int(y1) - 10), cv.FONT_HERSHEY_SIMPLEX, 0.5, (10, 3, 7), 1)
            return frame





"""
NOTES: 
i removed the color detection stuff because this is a the tracking node not the detection node and hence we don't want any detections here
removed the opencv stuff because this node takes all the detection from another node and doesn't detect stuff itself
please update the track method to include the correct tracking behaviour i tried to look for a function with the name "track_objects" but couldn't find any so please fix it
"""
