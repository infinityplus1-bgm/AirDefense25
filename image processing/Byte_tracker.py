import cv2 as cv
import numpy as np
from ultralytics import YOLO

class track:
    def __init__(self, video_source, tracker, model_path):
        """Initialize YOLO model, video source, and tracker."""
        self.model = YOLO(model_path)
        self.video = video_source
        self.video_capture = cv.VideoCapture(video_source)
        if not self.video_capture.isOpened():
            print("Error: Video file not opened.")
            return
        self.mot_tracker = tracker
    def track_objects(self, frame):
        """Track objects using the SORT tracker per frame."""
        results = self.model.track(frame, persist=True, tracker=self.mot_tracker, conf=0.53, iou=0.3, agnostic_nms=True, classes=[0])
        return results, frame
    def release(self):
        """Release video and windows."""
        self.video_capture.release()
        cv.destroyAllWindows()

tracker = track(r'Assets\ballon3.mp4', "bytebytetrack.yaml", r'weights\best.pt')

while True:
    tracking_list,frame = tracker.track_objects()
    if frame is None:
        break

    # Exit the loop when 'q' is pressed
    if cv.waitKey(20) & 0xFF == ord('q'):
        break
tracker.release()