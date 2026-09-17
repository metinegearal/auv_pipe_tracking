# Changelog

## 2026-09-17

- Added the `TORCH_INDEX_URL` Docker build argument and Compose passthrough so the PyTorch CUDA wheel channel can be selected for the host GPU and NVIDIA driver.
- Documented how to build with an alternate PyTorch CUDA channel, including stable CUDA 12.4 wheels.
- Documented the BauRov-v2.3.0 simulation world asset and automated host-side installation commands.

## 2026-09-15

- Added continuous trajectory optimization to the coordinate-free planner using dynamic look-ahead, cubic Bezier spatial smoothing, and exponential temporal smoothing.
- Added the `pipe_track_mission` package with Behavior Tree mission termination for reactive and map-based modes.
- Added the initial `pipe_track_evaluation` package for ground-truth versus estimated odometry metrics and trajectory plots.
- Added benchmark result figures and documented reported trajectory RMSE values in the README.