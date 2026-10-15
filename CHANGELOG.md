# Changelog

## 2026-10-15
- Added a discrete LQR controller with position and velocity feedback, waypoint braking, eight-thruster allocation, force limiting, and sensor timeout checks; selected it in navigation bringup.
- Exposed DLQR model and tuning weights in bringup configuration and configured navigation to use world-coordinate waypoints.
- Added nominal DLQR analysis through helper modules attached to the existing evaluation node: damping ratios, peak response, 63% crossing time, 2% settling time, 10–90% rise time, controllability/observability ranks and singular values, phase/gain margins, and sensitivity peak.
- Added a single DLQR text report and a combined sensitivity/Nichols plot page using the controller's published model and gains.
- Made tracking summaries and CSV data refresh every 5 seconds, retained final report/plot generation on mission completion or Ctrl+C, handled missing estimator samples, and added retries for failed final saves.
- Fixed map target selection to compare frontier cells and vehicle position in consistent world units while preserving target smoothing and the current-position fallback when exploration is empty.
- Re-enabled the visual odometry node in navigation bringup and VO measurement processing in the estimator.
- Added numerical DLQR, report lifecycle, and map target regression tests, plus controller and evaluation documentation.

## 2026-09-22
- Absolute Trajectory Error (ATE) RMSE calculation in metrics_node to compare Estimated Odometry against Ground-Truth.
- GTSAM graph based location estimation/fusion is added to the system (`pipe_track_estimation` pkg)
- Visual odometry package added to the system (`pipe_track_vo` pkg)
- Nodes that uses Ground truth now use estimated location which creates more real results. 

## 2026-09-18
- Resolved high-latency oscillation bug (#1) by replacing iterative image skeletonization with a fast Distance Transform centerline extraction, reducing planning latency from >1000ms to <10ms.
- Fixed ROS 2 subscriber queue backlog in the perception node by reducing the camera mask queue size to 1, guaranteeing real-time frame evaluation.
- Implemented exact end-to-end pipeline latency tracking by propagating camera frame timestamps through `geometry_msgs/msg/PointStamped`.
- Tuned coordinate-free planner `angle_weight` from 0.5 to 0.2, improving mean AUV tracking velocity from 0.65 m/s to 0.73 m/s and reducing hesitation on sharp turns.

## 2026-09-17
- Added the `TORCH_INDEX_URL` Docker build argument and Compose passthrough so the PyTorch CUDA wheel channel can be selected for the host GPU and NVIDIA driver.
- Documented how to build with an alternate PyTorch CUDA channel, including stable CUDA 12.4 wheels.
- Documented the BauRov-v2.3.0 simulation world asset and automated host-side installation commands.

## 2026-09-15
- Added continuous trajectory optimization to the coordinate-free planner using dynamic look-ahead, cubic Bezier spatial smoothing, and exponential temporal smoothing.
- Added the `pipe_track_mission` package with Behavior Tree mission termination for reactive and map-based modes.
- Added the initial `pipe_track_evaluation` package for ground-truth versus estimated odometry metrics and trajectory plots.
- Added benchmark result figures and documented reported trajectory RMSE values in the README.
