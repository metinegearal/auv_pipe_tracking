# Changelog

## 2026-09-15

- Added continuous trajectory optimization to the coordinate-free planner using dynamic look-ahead, cubic Bezier spatial smoothing, and exponential temporal smoothing.
- Added the `pipe_track_mission` package with Behavior Tree mission termination for reactive and map-based modes.
- Added the initial `pipe_track_evaluation` package for ground-truth versus estimated odometry metrics and trajectory plots.
- Added benchmark result figures and documented reported trajectory RMSE values in the README.