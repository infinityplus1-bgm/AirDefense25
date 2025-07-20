import cv2 as cv
import numpy as np
from ultralytics import YOLO


class image_processing:
    def __init__(self, video_source, tracker, model_path):
        """Initialize YOLO model, video source, and tracker."""
        self.model = YOLO(model_path)
        self.video = video_source
        self.video_capture = cv.VideoCapture(video_source)
        if not self.video_capture.isOpened():
            print("Error: Video file not opened.")
            return
        self.mot_tracker = tracker

    def detect_objects(self, frame, confidence_threshold=0.3):
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
    
    def detect_shape(self, frame):
        gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        blurred = cv.GaussianBlur(gray, (5, 5), 0)
        edged = cv.Canny(blurred, 50, 150)
        contours, _ = cv.findContours(edged, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        shapes_detectioons = []
        for contour in contours:
             # Approximate the contour
            epsilon = 0.04 * cv.arcLength(contour, True)
            approx = cv.approxPolyDP(contour, epsilon, True)

            # Get bounding box for labeling
            x, y, w, h = cv.boundingRect(approx)

            shape = "Unidentified"
            if len(approx) == 3:
                shape = "Triangle"
            elif len(approx) == 4:
                aspect_ratio = w / float(h)
                shape = "Square" if 0.95 < aspect_ratio < 1.05 else "Rectangle"
            elif len(approx) > 4:
                shape = "Circle"
            
            shapes_detectioons.append([shape, x, y, w, h])
         
        return shapes_detectioons
        
    def track_objects(self, frame):
        """Track objects using the SORT tracker per frame."""
        results = self.model.track(frame, persist=True, tracker=self.mot_tracker, conf=0.53, iou=0.3, agnostic_nms=True, classes=[0])
        return results

    def find_Red_Balloon(self, tracked_list, frame):
        """Find the red balloon in the detections."""
        red_balloon_detections = []
        frame_height, frame_width = frame.shape[:2]

        for r in tracked_list:
            for box in r.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
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
    
    def detect_qr_code(self, frame):
        qcd = cv.QRCodeDetector()
        retval, decoded_info, points, straight_qrcode = qcd.detectAndDecodeMulti(frame)
        return decoded_info, points

    def process_frame(self, frame):
        frame = cv.resize(frame, (1023, 1023))  # Resize to 1023x1023
        #frame = cv.rotate(frame, cv.ROTATE_90_CLOCKWISE)
        """Process frame: detect and track separately."""
        detections = self.detect_objects(frame)
        tracker_results = self.track_objects(frame)
        tracked_red_balloons = self.find_Red_Balloon(tracker_results, frame)
        decoded_info, points = self.detect_qr_code(frame) 
        shape_detections = self.detect_shape(frame)
       
        self.tracked_red_balloons = tracked_red_balloons  # Exposed for external access

        for r in tracker_results:
            for box in r.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                track_id = int(box.id[0])
                conf = float(box.conf[0])

                cv.putText(frame, f'ballon {int(track_id)}', (int(x1), int(y1) - 10), cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

                target_detection = [x1, y1, x2, y2]
                if any(self.is_close(target_detection, det[:4]) for det in tracked_red_balloons):
                    cv.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 0, 255), 2) 
        if points is not None:
            for point in points:
                if point is not None and len(point) == 4:
                    pt1 = tuple(map(int, point[0][0]))
                    pt2 = tuple(map(int, point[0][2]))
                    cv.rectangle(frame, pt1, pt2, (255, 0, 0), 2)
                    cv.putText(frame, decoded_info[0], pt1, cv.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv.LINE_AA) 
        
        for shape in shape_detections:
            shape_name, x, y, w, h = shape
            cv.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv.putText(frame, shape_name, (x, y - 10), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            
        return frame, tracker_results, tracked_red_balloons

    def get_results(self):
        """Fetch results: detect and track objects, process frame."""
        ret, frame = self.video_capture.read()
        if not ret:
            return None, None, None
        
        frame, tracker_results, tracked_red_balloons = self.process_frame(frame)
        return frame, tracker_results, tracked_red_balloons

    def release(self):
        """Release video and windows."""
        self.video_capture.release()
        cv.destroyAllWindows()

# Initialize the detector with video source, tracker, and model path
detector = image_processing(r'Assets\ballon5.mp4', "botsort.yaml", r'weights\best.pt')

while True:
    frame, tracker_results, tracked_red_balloons = detector.get_results()
    if frame is None:
        break

    # Display the frame
    cv.imshow("Frame", frame)
    cv.imwrite("image processing\yolo-main\screenshots", frame)  # Save the output frame

    # Print tracker results and tracked red balloons
    print("tracker_results:", tracker_results)
    print("tracked_red_balloons:", tracked_red_balloons)

    # Exit the loop when 'q' is pressed
    if cv.waitKey(20) & 0xFF == ord('q'):
        break

detector.release()
