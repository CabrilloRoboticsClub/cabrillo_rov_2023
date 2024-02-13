# H264 Recording
The `h264_recording` bag contains a recording of 915 messages from the `/image_raw/h264` topic for a duration of 30 seconds. It's purpose is to replay H264 encoded camera feed to enable remote development without the need for a physical camera.
![Image](/doc/img/h264_recording.gif)

## Usage
### H264 encoded messages
To play just the H264 encoded messages play the ROS bag. 
```console
ros2 bag play bags/h264_recording
```

### RAW messages
To access raw values from from the recording start by starting the re-publisher.
```console
ros2 run image_transport republish h264 raw --ros-args -r in/h264:=/image_raw/h264 -r out:=/repub_raw
```
Play the ROS bag. 
```console
ros2 bag play bags/h264_recording
```
Messages are published to the `/repub_raw` topic as RGB values.

### Play in RQT
Execute the steps described in RAW messages then open rqt with the following command. Note you may need to unset the `GTK_PATH` environment variable if VS Code was installed with Snap.
```sh
rqt
```
This should open an RQT window. From the `Plugins` dropdown open `Visualization > Image Viewer`. Select the `/repub_raw` topic. The video from the ROS bag should be displayed.