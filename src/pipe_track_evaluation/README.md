# DLQR feature evaluation

The same evaluation node updates `metrics_summary.txt` and `time_series_data.csv`
every 5 seconds while trajectory samples are available. Configure the positive
`metrics_update_period` parameter under `evaluation_system.ros__parameters` in
bringup to change this interval. Receiving `/movement/finished_execution=true`
or pressing Ctrl+C also saves the final report and tracking plot. A temporarily
unchanged map is not itself a completion event: the mission waits for an empty
frontier list. Missing estimator samples are shown as N/A in the summary.

The existing `evaluation_system` node handles both tracking metrics and DLQR
reporting in `pipe_track_navi.launch.py`. DLQR subscription handling lives in
`dlqr_evaluation.py`, and numerical analysis and plotting live in
`dlqr_analysis.py`; these are helper modules, not additional nodes.
The controller publishes its actual discrete plant
and feedback gains on `/motion/dlqr_model` with reliable, transient-local QoS.
The evaluation node receives the model even when started later, and writes:

- `results/dlqr_features.txt`: one report with four sections covering the model,
  damping and step response, controllability/observability, and frequency margins.
- `results/dlqr_sensitivity_nichols.png`: one page containing sensitivity and
  Nichols plots for both position and velocity feedback, with X/Y/Z curves.

Build and run from the workspace after sourcing ROS:

```bash
colcon build --packages-select pipe_track_control pipe_track_evaluation pipe_track_bringup
source install/setup.bash
ros2 launch pipe_track_bringup pipe_track_navi.launch.py
```

To start the evaluation system manually when it is not already running:

```bash
ros2 run pipe_track_evaluation evaluation_system --ros-args -p results_dir:=results
```

The output directory is relative to the launch working directory unless absolute.
An identical model is processed once; a changed model after a controller restart
regenerates these files. No motion or test commands are sent to the vehicle.

The unit position step uses the state feedback gain and zero target velocity.
The unit velocity step includes the controller's damping feedforward. Both modes
are reported because navigation can switch between them. Step metrics are peak
output, first 63% crossing, sampled 2% settling time, and interpolated 10–90% rise
time. The simulation horizon is at least 30 s and 12 slow-pole time constants,
capped at 120,000 samples; a response that has not settled is marked N/A.

Damping ratios come from `s = log(z)/dt` for each closed-loop pole. A 63% crossing
is not necessarily a physical time constant for the two-state position loop.
Controllability and observability report rank and singular values, with `C=I`
representing the position/velocity measurements assumed by the controller.
Velocity-mode analysis excludes unregulated position integrators.

For each independent axis, the force-input return ratio is
`L(z) = K(zI-A)^-1 B`, and sensitivity is `S(z) = 1/(1+L(z))`. Phase margin is
reported in degrees at unity gain; gain margin is reported as a factor and in dB
at negative-real loop response (-180 degrees modulo 360). The discrete Nyquist
endpoint is included. Sensitivity peak is the maximum magnitude of S from DC to
Nyquist, found using a combined logarithmic/linear sweep and local refinement.
These are nominal linear-model metrics, excluding saturation, yaw, transport
delays, waypoint shaping, and mode switching. Future coupled models are rejected
instead of silently assigning misleading per-axis margins.

Plots start two decades below the slowest closed-loop pole frequency for
readability; the numerical analysis still covers DC through Nyquist.
Margin conventions follow the
[Python Control stability-margin definitions](https://python-control.readthedocs.io/en/stable/generated/control.stability_margins.html).

Numerical tests (without a running ROS graph):

```bash
PYTHONPATH=src/pipe_track_evaluation python3 -m pytest -q src/pipe_track_evaluation/test/test_dlqr_analysis.py
```
