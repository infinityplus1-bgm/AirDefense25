import cv2 as cv
import numpy as np

class tracker:
    def __init__(self , tracker):
        self.mot_tracker = tracker
        self.detections = detections
    def find_tracking_list(self):
        return self.mot_tracker.update(self.detections)

    def is_close(self, box1, box2, threshold=15):
        """Check if two boxes are close enough (avoiding exact match problems)."""
        return all(abs(box1[i] - box2[i]) < threshold for i in range(4)) # FIXME: i dont think this actually solves the exact match problem ??

    def track(self , detections):
        """Process frame: detect and track separately."""

        tracker_results = self.track_objects(detections) # FIXME: where is this function ??

            #* RESULT: after talking with abd eid he said that it mostly does and hence we will continue with it until we find a better solution
            target_detection = [x1, y1, x2, y2]
            if any(self.is_close(target_detection, det[:4]) for det in tracked_red_balloons):
                # FIXME: this node doesn't display anything and just returns the tracked objects so no need to draw anything please fix and move this code elsewhere
                cv.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 0, 255), 2) 

        #  TODO : might not need the frame back only the tracker coordinates
        return frame, tracker_results, tracked_red_balloons



"""
NOTES: 
i removed the color detection stuff because this is a the tracking node not the detection node and hence we don't want any detections here
removed the opencv stuff because this node takes all the detection from another node and doesn't detect stuff itself
please update the track method to include the correct tracking behaviour i tried to look for a function with the name "track_objects" but couldn't find any so please fix it
"""