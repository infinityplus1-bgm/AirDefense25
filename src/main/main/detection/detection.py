
import numpy as np
from typing import Any # Import Any if the exact model type isn't known or fixed

class Detector:
    """
    A class for performing object detection using a provided model.

    Attributes:
        model: The object detection model instance to be used for inference.
               Expected to be callable and compatible with the YOLO interface
               (i.e., accepts a frame and returns detection results).
    """
    def __init__(self, model: Any):
        """
        Initializes the Detect class with a detection model.

        Args:
            model (Any): The object detection model to use. The exact type depends
                         on the specific library (e.g., Ultralytics YOLO model object).
        """
        self.model: Any = model # Added type annotation for the instance attribute

    def detect_objects(self, frame: np.ndarray, confidence_threshold: float = 0.53) -> np.ndarray:
        """
        Performs object detection on a single image frame using the initialized model.

        Args:
            frame (np.ndarray): The input image frame as a NumPy array (typically BGR or RGB).
            confidence_threshold (float, optional): The minimum confidence score required
                                                    for a detection to be included in the
                                                    results. Defaults to 0.53.

        Returns:
            np.ndarray: A NumPy array containing the detected bounding boxes and their
                        confidence scores. Each row represents a detection and has the
                        format [x1, y1, x2, y2, confidence], where (x1, y1) is the
                        top-left corner, (x2, y2) is the bottom-right corner, and
                        confidence is the detection probability. Returns an empty array
                        if no objects are detected above the threshold.
                        The coordinates (x1, y1, x2, y2) are integers, and the
                        confidence is a float.
        """
        # YOLO Detection to get bounding boxes.
        results = self.model(frame, stream=True) # Assumes model is callable
        detections = []

        for r in results:
            # Assumes 'r' has a 'boxes' attribute compatible with Ultralytics YOLO results
            for box in r.boxes:
                # Extract bounding box coordinates (xyxy format) and confidence
                x1, y1, x2, y2 = map(int, box.xyxy[0]) # box.xyxy[0] gives coordinates
                conf = float(box.conf[0])             # box.conf[0] gives confidence

                # Filter detections based on the confidence threshold
                if conf >= confidence_threshold:
                    detections.append([x1, y1, x2, y2, conf])

        # Convert the list of detections to a NumPy array
        detections_array = np.array(detections, dtype=np.float32) # Specify dtype for consistency
        return detections_array