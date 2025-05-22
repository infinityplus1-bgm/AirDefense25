import cv2 as cv
import numpy as np

class Tracker:
    def __init__(self, tracker) -> None:
        """
        Initialize the Tracker class.

        Args:
            tracker: SORT tracker or any tracking algorithm instance.
            weights (str): Path to the model weights to load.
        """
        self.mot_tracker = tracker

    def track_objects(self, detections: np.ndarray) -> np.ndarray:
        """
        Update the tracker with new detections and return tracked objects.

        Args:
            detections (np.ndarray): Array of detections in the format [x1, y1, x2, y2, conf].

        Returns:
            np.ndarray: Array of tracked objects in the format [x1, y1, x2, y2, track_id].
        """
        if detections is None or len(detections) == 0:
            return np.empty((0, 5), dtype=np.float32)
        tracks = self.mot_tracker.update(detections)
        return np.array(tracks, dtype=np.float32)

    def process_frame(self, tracked_objects: np.ndarray) -> np.ndarray:
        """
        Process the tracked objects to extract centers and track IDs.

        Args:
            tracked_objects (np.ndarray): Array of tracked objects [x1, y1, x2, y2, track_id].

        Returns:
            np.ndarray: Array of centers and IDs in the format [xa, ya, track_id].
        """
        centers_with_ids = []
        for obj in tracked_objects:
            x1, y1, x2, y2, track_id = obj
            xa = int((x1 + x2) / 2)
            ya = int((y1 + y2) / 2)
            centers_with_ids.append([xa, ya, int(track_id)])

        return np.array(centers_with_ids, dtype=np.int32)
