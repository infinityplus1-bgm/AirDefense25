from ultralytics import YOLO
import cv2 as cv
import numpy as np

class tracker:
    def __init__(self, model, video_source, tracker, weights):
        """Initialize YOLO model, video source, and tracker."""
        self.raw_model = model
        self.model = model(weights) # TODO:  i don't think the model needs weight check how SORT what does before by @abd eid and implement(done)
        # TODO:  change this because this takes ready detection from topic (done)
        
        self.mot_tracker = tracker
        
    def track_objects(self, detections: np.ndarray) -> np.ndarray:
        #TODO: instead of taking a full frame you will be getting the detection in the format (x1,y1,x2,y2,conf) so change accordingly (done)
        if detections is None or len(detections) == 0:
            return np.empty((0, 5), dtype=np.float32)
        tracks = self.mot_tracker.update(detections)
        return np.array(tracks, dtype=np.float32)
        
    def process_frame(self, tracked_objects: np.ndarray):
        """Process the array of tracked objects to get centers and track IDs."""
        centers_with_ids = []
        # TODO : we don't need to write to the frame as it only published the tracker results and the closest distance to the kill zone (done)
        for obj in tracked_objects:
            x1, y1, x2, y2, track_id = obj
            xa = int((x1 + x2) / 2)  # X center of the bounding box
            ya = int((y1 + y2) / 2)  # Y center of the bounding box
            centers_with_ids.append([xa, ya, track_id])  # Include track_id

        return np.array(centers_with_ids, dtype=np.int32)





"""
NOTES: 
i removed the color detection stuff because this is a the tracking node not the detection node and hence we don't want any detections here
removed the opencv stuff because this node takes all the detection from another node and doesn't detect stuff itself
please update the track method to include the correct tracking behaviour i tried to look for a function with the name "track_objects" but couldn't find any so please fix it
"""
