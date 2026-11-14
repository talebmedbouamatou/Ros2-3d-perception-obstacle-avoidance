# HP60C Depth Camera

## Camera reference

The project uses an **HP60C depth camera** as the depth-image source for the ROS 2 perception system.

## Depth image topic

The camera publishes depth images on:

```text
/ascamera_hp60c/camera_publisher/depth0/image_raw

The image encoding is 16UC1. Depth values are provided in millimeters and converted to meters in perception_3d.py.
Start the camera driver
ros2 launch ascamera hp60c.launch.py

Check the depth stream
ros2 topic hz /ascamera_hp60c/camera_publisher/depth0/image_raw

List available ROS 2 topics
ros2 topic list
