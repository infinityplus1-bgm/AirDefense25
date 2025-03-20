from ultralytics import YOLO
import cv2 as cv
import math
import numpy as np
from sort import *
tracked_dict = {}


class Detect:
    def __init__(self, video_source, model_path, tracked_dict={}):
        """Initialize the YOLO model and video source."""
        self.model = YOLO(model_path)
        self.video_capture = cv.VideoCapture(video_source)
        self.class_names = self.load_class_names()
        self.mot_tracker = Sort(max_age=20, min_hits=2, iou_threshold=0.3)
        self.tracked_dict = tracked_dict

    def load_class_names(self):
        """Loads the class names for YOLO detection."""
        return [
            "person", "bicycle", "ballon" ,"airplane", "bus", "train", "truck", "boat",
            "traffic light", "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat", "dog",
            "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella",
            "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball", "kite", "baseball bat",
            "baseball glove", "skateboard", "surfboard", "tennis racket", "bottle", "wine glass", "cup", "fork",
            "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange", "broccoli", "carrot", "hot dog",
            "pizza", "donut", "cake", "chair", "couch", "potted plant", "bed", "dining table", "toilet", "TV",
            "laptop", "mouse", "remote", "keyboard", "cell phone", "microwave", "oven", "toaster", "sink",
            "refrigerator", "book", "clock", "vase", "scissors", "teddy bear", "hair drier", "toothbrush"
        ]


                 
    def process_frame(self, frame):
        """Processes a single frame and applies YOLO object detection."""
        frame = cv.resize(frame, (1023, 1023))  # Resize to 320x240
        frame = cv.rotate(frame, cv.ROTATE_90_CLOCKWISE)  # Rotate 90 degrees clockwise
        results = self.model(frame, stream=True)
        detections = np.empty((0, 5))

        for r in results:
            for box in r.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                conf = float(box.conf[0])
                cls = int(box.cls[0])
                
                if conf >= 0.53 :  # Class 2 ==ca (removed)
                    current_Array = np.array([x1, y1, x2, y2, conf])
                    detections = np.vstack((detections, current_Array))

        if len(detections) > 0:
            tracker_results = self.mot_tracker.update(detections)
            for tracked in tracker_results:
                X1, Y1, X2, Y2, ID = tracked
                cv.putText(frame, f'{self.class_names[2]} {int(ID)}',
                           (int(X1), int(Y1) - 10), cv.FONT_HERSHEY_SIMPLEX, 0.5, (10, 3, 7), 1)
            self.tracked_dict = {int(row[-1]): row[:-1].tolist() for row in tracker_results}
        
        return frame
    def get_Detection_list(self):
        return self.tracked_dict

    def run(self):
        """Runs the object detection on the video feed."""
        while True:
            is_true, frame = self.video_capture.read()
            if not is_true:
                break

            frame = self.process_frame(frame)
            cv.imshow('YOLO Object Detection', frame)
            print(self.get_Detection_list()) 

            if cv.waitKey(20) & 0xFF == ord('d'):
                break

        self.video_capture.release()
        cv.destroyAllWindows()


# Running the YOLO Object Detection
detector = Detect(r'Assets\Ballons.mp4', r'weights\best.pt',tracked_dict)
'''1) when writing the path of the video and the model, it is important to use the raw string format
2) either write the whole path starting with D:/  or write it starting from inside the code file'''
# r'' indicates it is a raw string , such that the backlashes do not produce a syntax
detector.run()

