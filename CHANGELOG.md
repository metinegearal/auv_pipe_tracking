# Changelog

## 2026-09-18

- Resolved high-latency oscillation bug (#1) by replacing iterative image skeletonization with a fast Distance Transform centerline extraction, reducing planning latency from >1000ms to <10ms.
- Fixed ROS 2 subscriber queue backlog in the perception node by reducing the camera mask queue size to 1, guaranteeing real-time frame evaluation.
- Implemented exact end-to-end pipeline latency tracking by propagating camera frame timestamps through `geometry_msgs/msg/PointStamped`.
- Tuned coordinate-free planner `angle_weight` from 0.5 to 0.2, improving mean AUV tracking velocity from 0.65 m/s to 0.73 m/s and reducing hesitation on sharp turns.

## 2026-09-15

- Added continuous trajectory optimization to the coordinate-free planner using dynamic look-ahead, cubic Bezier spatial smoothing, and exponential temporal smoothing.
- Added the `pipe_track_mission` package with Behavior Tree mission termination for reactive and map-based modes.
- Added the initial `pipe_track_evaluation` package for ground-truth versus estimated odometry metrics and trajectory plots.
- Added benchmark result figures and documented reported trajectory RMSE values in the README.