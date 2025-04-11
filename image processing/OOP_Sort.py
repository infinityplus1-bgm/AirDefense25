# TODO : ask whether we still need it or should it be removed @abd eid

from ultralytics import YOLO
import cv2 as cv
import numpy as np
from sort import *


class Detect:
    def __init__(self, video_source, model_path,):
        """Initialize YOLO model, video source, and tracker."""
        self.model = YOLO(model_path)
        self.video_capture = cv.VideoCapture(video_source)
        self.mot_tracker = Sort(max_age=120, min_hits=3, iou_threshold=0.3)
        #max_Age is the maximum time of how many frames can a tracked object be lost without being deleted from the list of tracked objects

    def detect_objects(self, frame, confidence_threshold=0.53):
        """YOLO Detection to get bounding boxes."""
        results = self.model(frame, stream=True)
        detections = []

        for r in results:
            for box in r.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                conf = float(box.conf[0])

                if conf >= confidence_threshold:
                    #np.vstack() was expensive as it allocates new memory every time
                    detections.append([x1, y1, x2, y2, conf])
        detections = np.array(detections)
        #https://www.analyticsvidhya.com/blog/2024/02/python-list-to-numpy-arrays/

        return detections

    def track_objects(self, detections):
        """Track objects using the SORT tracker."""
        return self.mot_tracker.update(detections)
    
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

             #we need to understand more on the HSV color space and how did they tighten it.
        return red_balloon_detections
        '''The bounding box coordinates from the tracker go out of frame bounds (e.g., negative or exceed image dimensions).

The balloon leaves the scene or is lost, but the tracker still returns stale coordinates for a few frames due to max_age.

thus, we had to make x1,y1,x2,y2 limited to the frame size such that we avoid foolish glitches. "need more understanding of that part"'''
    def is_close(self, box1, box2, threshold=15):
        """Check if two boxes are close enough (avoiding exact match problems)."""
        return all(abs(box1[i] - box2[i]) < threshold for i in range(4))
    # we used is_close as similar to IOU(   Intersection Over Union) so that it is not that 
    #strict and we can get the red balloon even if it is not exactly the same as the one in the tracker list.

    ### is_close was very good in keeping track with the red ballon and not falling strict to the exact match of the coordinates
    def process_frame(self, frame):
        """Process frame: detect and track separately."""
        start = cv.getTickCount()

        detections = self.detect_objects(frame)
        tracker_results = self.track_objects(detections)
        tracked_red_balloons = self.find_Red_Balloon(tracker_results, frame)

        self.tracked_red_balloons = tracked_red_balloons  # Exposed for external access

        for tracked in tracker_results:
            x1, y1, x2, y2, track_id = tracked
            cv.putText(frame, f'ID {int(track_id)}',
                       (int(x1), int(y1) - 10), cv.FONT_HERSHEY_SIMPLEX, 0.5, (10, 3, 7), 1)

            target_detection = [x1, y1, x2, y2]
            if any(self.is_close(target_detection, det[:4]) for det in tracked_red_balloons):
                cv.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 0, 255), 2)

        # Add FPS counter (optional)
        '''      fps = cv.getTickFrequency() / (cv.getTickCount() - start)
        cv.putText(frame, f'FPS: {fps:.2f}', (10, 30),
                   cv.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)'''

        return frame, tracker_results, tracked_red_balloons
   
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

#we need to understand how does video capture works so that we are able to manipulate it for later applicatoins



# Run the detector
detector = Detect(r'Assets\ballon3.mp4', r'weights\best.pt')
while True:
    frame, tracker_results, tracked_red_ballons = detector.get_results()
    if frame is None:
        break
    cv.imshow("Frame", frame)
    print("tracker_results:", tracker_results)
    print("tracked_red_ballons:", tracked_red_ballons)
    if cv.waitKey(20) & 0xFF == ord('q'):
        break

detector.release()
#the mistake that was made here is that all the real time process of both detection and tracking were made inside the class, such 
#that list is not produced until the end of the video



'''the error with getting the red ballon if the ballon_list is updated
from the yolo detection list instead of the tracker list is that the yolo detection list is not updated

You correctly detect and track objects.

You filter red balloons from the YOLO detections, not from the tracked results.

SORT (your tracker) assigns new bounding box coordinates that may not match the original detection coordinates exactly,
 even by a pixel — causing np.array_equal(...) to fail on subsequent frames.'''

'''need to understand that code: 

start = cv.getTickCount()
# ... run detection/tracking
fps = cv.getTickFrequency() / (cv.getTickCount() - start)
cv.putText(frame, f'FPS: {fps:.2f}', (10, 30), cv.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

'''

