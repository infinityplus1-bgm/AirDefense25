from ultralytics import YOLO
import cv2 as cv
import numpy as np

class Detect:
    def __init__(self, detection_model, weights, video_path):
        
        self.weights = weights
        self.model = detection_model(self.weights )
        self.raw_video = video_path
        self.video = cv.VideoCapture(video_path)
    
    def detect_objects(self, frame, confidence_threshold=0.53):
        """YOLO Detection to get bounding boxes."""
        results = self.model(frame, stream=True)
        detections = []

        for r in results:
            for box in r.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                conf = float(box.conf[0])

                if conf >= confidence_threshold:
                    detections.append([x1, y1, x2, y2, conf])

        return detections
    def find_Red_Balloon(self, detection_list, frame):
        """Find the red balloon in the detections."""
        red_balloon_detections = []
        frame_height, frame_width = frame.shape[:2]

        for detected in detection_list:
                x1, y1, x2, y2 = map(int, detected[:4])
                tracked = [x1,y1,x2,y2]

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

                #we need to understand more on the HSV color space and how did they tighten it.
        return red_balloon_detections

    def is_close(self, box1, box2, threshold=15):
        """Check if two boxes are close enough (avoiding exact match problems)."""
        return all(abs(int(box1[i]) - int(box2[i])) < threshold for i in range(4))
    def process_frame(self, frame):
        """Process a single frame for detection and tracking."""
        detections = self.detect_objects(frame)
        red_balloon_detections = self.find_Red_Balloon(detections, frame)
        for r in red_balloon_detections:
            x1, y1, x2, y2 = map(int, r)
            target_detection = [x1, y1, x2, y2]
            if any(self.is_close(target_detection, det[:4]) for det in red_balloon_detections):
                    cv.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 0, 255), 2)
            return frame
        
    def get_results(self):
        ret, frame = self.video.read()
        if not ret:
            return None
        detections = self.detect_objects(frame)
        red_balloon_detections = self.find_Red_Balloon(detections, frame)
        frame = self.process_frame(frame)
        return detections, red_balloon_detections, frame
    
    def release(self):
        """Release video and windows."""
        self.video_capture.release()
        cv.destroyAllWindows()

detector = Detect(YOLO, r'weights\best.pt', r'Assets\ballon3.mp4')
while True:
    detections, red_balloon_detections, frame = detector.get_results()
    if frame is None:
        break

    # Display the frame
    cv.imshow("Frame", frame)


    # Exit the loop when 'q' is pressed
    if cv.waitKey(20) & 0xFF == ord('q'):
        break

detector.release()
 

# 