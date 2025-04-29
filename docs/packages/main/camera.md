This directory provides 2 different nodes:
1. `camera`
2. `camera_test`

## camera

This is the main node that actually interact with the whole system and that the `detection` node takes its data from...

The node publishes the captured frame to `/camera/image_raw` 

the node is built with python and support changing the frame rate as well as the video source through the command-line interface rather than by changing the code itself


### USAGE:
#### to get help one how to use
```shell
ros2 run main camera -h
```
this should provide you with a helpful manual for how to use the node 

#### run using the default webcam at 30 fps 
```shell
ros2 run main camera -v 0 -t 0.033
```

#### run using `arducam` at 30 fps
```shell
ros2 run main camera -v arducam -t 0.033
```

#### run using a video file at 60 fps
```shell
ros2 run main camera -v /path/to/file.mp4 -t 0.016
```


### Note about arducam :
the default configuration for opencv wouldn't let us get 30fps out of the camera even though we know it has that capability. After digging deeper we found that this camera uses 2 different codecs namely `YUYV` and `MJPG` , it turns out opencv uses `YUYV` by default which only supports 10 fps. so we would check if the argument given is `arducam` and if so we would change the codec to `MJPG` to be able to get 30 fps

## camera_test

This node is just used to test whether the camera node is properly working or not 

### USAGE:

#### display the output of camera node
```shell
ros2 run main camera_test
```
