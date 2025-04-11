from ultralytics import YOLO
import cv2 as cv
import numpy as np
from sort import *

class tracker:
    def __init__(self, video_path,detections,tracker=Sort(max_age=120, min_hits=3, iou_threshold=0.3)):
        self.mot_tracker = tracker
        self.video = cv.VideoCapture(video_path)
        self.detections = detections
    def find_tracking_list(self):
        return self.mot_tracker.update(self.detections)
    def find_Red_Balloon(self, tracked_list, frame):
        """Find the red balloon in the detections."""
        red_balloon_detections = []
        frame_height, frame_width = frame.shape[:2]

        for tracked in tracked_list:
            x1, y1, x2, y2, ID = tracked

            # Clamp bounding box coordinates within image bounds
            x1 = max(0, min(int(x1), frame_width - 1))
            y1 = max(0, min(int(y1), frame_height - 1))
            x2 = max(0, min(int(x2), frame_width - 1))
            y2 = max(0, min(int(y2), frame_height - 1))

            if x2 <= x1 or y2 <= y1:
                continue  # Skip invalid ROI

            roi = frame[y1:y2, x1:x2]

            if roi.size == 0:
                continue  # Skip empty ROI

            # Convert ROI to HSV color space
            hsv_roi = cv.cvtColor(roi, cv.COLOR_BGR2HSV)

            lower_red1 = np.array([0, 150, 120])
            upper_red1 = np.array([10, 255, 255])
            lower_red2 = np.array([160, 150, 120])
            upper_red2 = np.array([180, 255, 255])
            mask1 = cv.inRange(hsv_roi, lower_red1, upper_red1)
            mask2 = cv.inRange(hsv_roi, lower_red2, upper_red2)
            mask = cv.bitwise_or(mask1, mask2)

            red_ratio = np.sum(mask > 0) / (roi.shape[0] * roi.shape[1])
            if red_ratio > 0.2:
                red_balloon_detections.append(tracked)

             # TODO : we need to understand more on the HSV color space and how did they tighten it.
        return red_balloon_detections
    def is_close(self, box1, box2, threshold=15):
        """Check if two boxes are close enough (avoiding exact match problems)."""
        return all(abs(box1[i] - box2[i]) < threshold for i in range(4))
    def process_frame(self, frame):
        """Process frame: detect and track separately."""


        detections = self.detect_objects(frame) # TODO: we should take detection from ros topic not from camera or video
        tracker_results = self.track_objects(detections) # FIXME: what the heck is this function ??
        tracked_red_balloons = self.find_Red_Balloon(tracker_results, frame)

        self.tracked_red_balloons = tracked_red_balloons  # Exposed for external access

        for tracked in tracker_results:
            x1, y1, x2, y2, track_id = tracked
            cv.putText(frame, f'ID {int(track_id)}',
                       (int(x1), int(y1) - 10), cv.FONT_HERSHEY_SIMPLEX, 0.5, (10, 3, 7), 1)



            # TODO : check if this actually solves the id change problem
            target_detection = [x1, y1, x2, y2]
            if any(self.is_close(target_detection, det[:4]) for det in tracked_red_balloons):
                cv.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 0, 255), 2)

        #  TODO : might not need the frame back only the tracker coordinates
        return frame, tracker_results, tracked_red_balloons


    # TODO : change this so that it take from ros topic not from camera or video
    def get_results(self):
        ret,frame = self.video_capture.read()
        if not ret:
            return None, None, None
        else:
            frame, tracker_results, tracked_red_ballons = self.process_frame(frame)
            return frame, tracker_results, tracked_red_ballons    
    def release(self):
        """Release video and windows."""
        self.video_capture.release()
        cv.destroyAllWindows()


 # TODO : remove this make it only a class file
tracker = tracker(r'Assets\ballon3.mp4', r'weights\best.pt')
while True:
    frame, tracker_results, tracked_red_ballons = tracker.get_results()
    if frame is None:
        break
    cv.imshow("Frame", frame)
    print("tracker_results:", tracker_results)
    print("tracked_red_ballons:", tracked_red_ballons)
    if cv.waitKey(20) & 0xFF == ord('q'):
        break
tracker.release()