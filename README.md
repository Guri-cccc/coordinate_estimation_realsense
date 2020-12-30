# coordinate_estimation_realsense

Coordinate estimation (depth estimation) using intel realsense

This package estimates relative coordination of bounding box (/darknet_ros/bounding_boxes) using realsense.

The resolution of the input depth image (/camera/depth/image_rect_raw) has to be 640 * 480.

Start:

rosrun coordinate coordinate_estimation
