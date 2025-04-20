# TODO: subscribe to the camera/image_raw topic and convert using cv_bridge (check camera_test)
# TODO: create a new model using YOLO(best.py) (check how it was done before using github)
# TODO: create new object from the detector class and pass the model as argument
# TODO: call the detector object's detect_object method and save the response in a variable
# TODO: research how to set the publisher such that it only publishes when it receives something from a topic it is subscribed to
# TODO: publish the detections to the "detections" topic
# TODO: draw the rectangles on the frame and publish to "detections/overlay"