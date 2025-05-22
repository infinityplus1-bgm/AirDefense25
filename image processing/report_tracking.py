from ultralytics import YOLO
import cv2 as cv
import numpy as np
from sort import *
import time

class Detect:
    # Added history_interval parameter
    def __init__(self, video_source, model_path, max_history_length=30, history_interval=5):
        """Initialize YOLO model, video source, tracker, and history storage."""
        self.model = YOLO(model_path)
        self.video_capture = cv.VideoCapture(video_source)
        self.mot_tracker = Sort(max_age=120, min_hits=3, iou_threshold=0.3)

        self.object_history = {} # Dictionary to store history: {track_id: [[x1, y1, x2, y2], ...]}
        self.max_history_length = max_history_length # Maximum number of past positions to store
        self.history_interval = history_interval   # Store history every N frames
        self.frame_count = 0                     # Counter for the current frame

    def detect_objects(self, frame, confidence_threshold=0.53):
        """YOLO Detection to get bounding boxes."""
        results = self.model(frame, stream=False)
        detections = []

        for r in results:
            if r.boxes and len(r.boxes) > 0:
                for box in r.boxes:
                    if box.xyxy.shape[0] > 0:
                        x1, y1, x2, y2 = map(float, box.xyxy[0].tolist())
                        conf = float(box.conf[0])

                        if conf >= confidence_threshold:
                            detections.append([x1, y1, x2, y2, conf])

        if detections:
            detections = np.array(detections)
        else:
            detections = np.empty((0, 5))

        return detections

    def track_objects(self, detections):
        """Track objects using the SORT tracker."""
        return self.mot_tracker.update(detections)

    def find_Red_Balloon(self, tracked_list, frame):
        """Find the red balloon in the tracked objects using color."""
        red_balloon_detections = []
        frame_height, frame_width = frame.shape[:2]

        for tracked in tracked_list:
            if len(tracked) == 5:
                x1, y1, x2, y2, ID = tracked

                x1_int = max(0, min(round(x1), frame_width - 1))
                y1_int = max(0, min(round(y1), frame_height - 1))
                x2_int = max(0, min(round(x2), frame_width - 1))
                y2_int = max(0, min(round(y2), frame_height - 1))

                if x2_int <= x1_int or y2_int <= y1_int:
                    continue

                roi = frame[y1_int:y2_int, x1_int:x2_int]

                if roi.size == 0 or roi.shape[0] <= 0 or roi.shape[1] <= 0:
                     continue

                try:
                    hsv_roi = cv.cvtColor(roi, cv.COLOR_BGR2HSV)

                    lower_red1 = np.array([0, 150, 120])
                    upper_red1 = np.array([10, 255, 255])
                    lower_red2 = np.array([160, 150, 120])
                    upper_red2 = np.array([180, 255, 255])

                    mask1 = cv.inRange(hsv_roi, lower_red1, upper_red1)
                    mask2 = cv.inRange(hsv_roi, lower_red2, upper_red2)
                    mask = cv.bitwise_or(mask1, mask2)

                    total_pixels = roi.shape[0] * roi.shape[1]
                    if total_pixels > 0:
                        red_ratio = np.sum(mask > 0) / total_pixels
                        if red_ratio > 0.2:
                            red_balloon_detections.append(tracked)
                except cv.error as e:
                    print(f"Error processing ROI for ID {ID}: {e}")
                    continue

        return red_balloon_detections

    def process_frame(self, frame):
        """Process frame: detect, track, update history, and draw."""
        start_time = time.time()
        self.frame_count += 1 # Increment frame counter

        detections = self.detect_objects(frame)
        tracker_results = self.track_objects(detections)

        current_track_ids = set()
        for tracked in tracker_results:
            x1, y1, x2, y2, track_id = tracked
            track_id_int = int(track_id)
            current_track_ids.add(track_id_int)

            if track_id_int not in self.object_history:
                self.object_history[track_id_int] = []

            # ONLY append to history if the current frame is the interval
            if self.frame_count % self.history_interval == 0:
                # Store coordinates as integers for drawing
                self.object_history[track_id_int].append([int(x1), int(y1), int(x2), int(y2)])

            # Keep history length limited
            # Note: max_history_length now refers to the number of *stored points*,
            # not necessarily the number of frames.
            if len(self.object_history[track_id_int]) > self.max_history_length:
                self.object_history[track_id_int].pop(0)

        # Find Red Balloons among the tracked objects
        tracked_red_balloons = self.find_Red_Balloon(tracker_results, frame.copy())
        self.tracked_red_balloons = tracked_red_balloons

        # Get red balloon IDs for easier lookup
        red_balloon_ids = {int(t[4]) for t in tracked_red_balloons}

        # Draw results (history and current position)
        for tracked in tracker_results:
            x1_curr, y1_curr, x2_curr, y2_curr, track_id = tracked
            track_id_int = int(track_id)

            base_color = (0, 0, 255) if track_id_int in red_balloon_ids else (255, 0, 0) # BGR format

            # Draw history trail (outlines)
            if track_id_int in self.object_history:
                history = self.object_history[track_id_int]
                # Draw older positions first with decreasing opacity/thickness
                for i, pos in enumerate(history):
                    hx1, hy1, hx2, hy2 = pos # History positions are already integers
                    # Calculate line thickness based on age (thinner for older)
                    # Adjusted thickness calculation as fewer points are drawn
                    # thickness = max(1, int(3 * (i + 1) / len(history))) if len(history) > 0 else 1 # Avoid division by zero

                    cv.rectangle(frame, (hx1, hy1), (hx2, hy2), (0 , 255 , 0), 2)


            # Draw the current bounding box (outline, more prominent)
            cx1, cy1, cx2, cy2 = map(int, [x1_curr, y1_curr, x2_curr, y2_curr])
            current_thickness = 3 if track_id_int in red_balloon_ids else 2 # Make current thicker
            cv.rectangle(frame, (cx1, cy1), (cx2, cy2), (0 , 255 , 0), current_thickness)

            # Draw ID text
            cv.putText(frame, f'ID {track_id_int}',
                       (cx1, cy1 - 10), cv.FONT_HERSHEY_SIMPLEX, 0.6, base_color, 2)


        # Add FPS counter
        end_time = time.time()
        fps = 1 / (end_time - start_time) if (end_time - start_time) > 0 else 0
        cv.putText(frame, f'FPS: {fps:.2f}', (10, 30),
                   cv.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        return frame, tracker_results, tracked_red_balloons

    def get_results(self):
        """Read frame and process it."""
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

# Run the detector
# You can adjust max_history_length (number of points) and history_interval (frames between points)
detector = Detect(r'Vid2.mp4', r'best.pt', max_history_length=50, history_interval=5) # e.g., store a point every 5 frames
while True:
    frame, tracker_results, tracked_red_ballons = detector.get_results()
    if frame is None:
        break

    cv.imshow("Frame", frame)

    if cv.waitKey(1) & 0xFF == ord('q'):
        break

detector.release()