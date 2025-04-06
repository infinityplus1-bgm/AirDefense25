from ultralytics import YOLO
import cv2 as cv
import numpy as np


class Detect:
    def __init__(self, model_path, video_path):
        self.model = YOLO(model_path)
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
                    #np.vstack() was expensive as it allocates new memory every time
                    detections.append([x1, y1, x2, y2, conf])
        detections = np.array(detections)
        #https://www.analyticsvidhya.com/blog/2024/02/python-list-to-numpy-arrays/
        return detections
    def get_results(self):
        ret, frame = self.video.read()
        if not ret:
            return None
        detections = self.detect_objects(frame)
        return detections
    def release(self):
        """Release video and windows."""
        self.video_capture.release()
        cv.destroyAllWindows()
        
detector = Detect(r'Assets\ballon3.mp4', r'weights\best.pt')
while True:
    detections = detector.get_results()
    if detections is None:
        break
    if cv.waitKey(20) & 0xFF == ord('q'):
        break
    detector.release()


