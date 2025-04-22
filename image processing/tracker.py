from ultralytics import YOLO
import cv2 as cv
import numpy as np

class tracker:
    def __init__(self, model, tracker, weights):
        """Initialize YOLO model, video source, and tracker."""
        self.raw_model = model
        self.model = model(weights)  # Assuming model doesn't need weight check
        
        # Use detection data instead of video capture, remove video source handling
        self.mot_tracker = tracker
        
    def track_objects(self, detections: np.ndarray) -> np.ndarray:
        # Use detection data directly instead of full frames
        if detections is None or len(detections) == 0:
            return np.empty((0, 5), dtype=np.float32)
        tracks = self.mot_tracker.update(detections)
        return np.array(tracks, dtype=np.float32)

    def process_frame(self, tracked_objects: np.ndarray):
        """Process the array of tracked objects to get centers and track IDs."""
        centers_with_ids = []
        for obj in tracked_objects:
            x1, y1, x2, y2, track_id = obj
            xa = int((x1 + x2) / 2)  # X center of the bounding box
            ya = int((y1 + y2) / 2)  # Y center of the bounding box
            centers_with_ids.append([xa, ya, track_id])  # Include track_id

        return np.array(centers_with_ids, dtype=np.int32)
