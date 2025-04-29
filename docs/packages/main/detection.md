This directory provides 2 different nodes:
1. `yolo_detection`
2. `detections_overlay_test`

## yolo_detection

This is the main node that subscribes to `/camera/image_raw` and get the frames from the camera for further processing.

The node uses `YOLO` using our fine-tuned model located at `best.pt` to detect balloons

The node publishes the detections to `/detections` and the overlay version where it draws boxes around detected objects to `/detectoins/overlay` 

### USAGE:
#### to get help one how to use
```shell
ros2 run main yolo_detection
```
this should start listening on the topic `/camera/image_raw`


### Note about training:
the training was done on a pre-labeled dataset from `roboflow` and was trained on `T4 TPU` using `google colab`

## detections_overlay_test

This node is just used to test whether the objects are being detected properly or not by subscribing to the overlay topic and displaying the frames 

### USAGE:

#### display the output of camera node
```shell
ros2 run main detections_overlay_test
```
