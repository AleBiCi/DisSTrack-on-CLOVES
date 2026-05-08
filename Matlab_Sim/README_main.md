# MATLAB Real-Time Tracking Pipeline

This directory contains the MATLAB implementation of the real-time localization and tracking
layer used in the CLOVES UWB experiments. The scripts consume the serial stream produced by
the EVB1000 mobile tag, reconstruct complete ranging rounds, apply calibration data, and run
a range-only Extended Kalman Filter (EKF) for two-dimensional tracking.

## Scope

The MATLAB layer is responsible for host-side estimation. It does not perform UWB ranging
itself; instead, it receives already computed SS-TWR ranges from the tag firmware. Its main
tasks are:

- parse `RANGING MEAS` lines emitted by the tag;
- group measurements by round;
- map UWB short addresses to anchor coordinates;
- compensate calibrated range biases;
- assign measurement covariance from per-node variance tables;
- initialize the state using weighted least squares (WLS);
- track the target with a unicycle range-only EKF;
- enforce map consistency through projection or rejection of invalid states;
- visualize the trajectory and optionally record map GIFs.

## Requirements

- MATLAB with support for `serialport`.
- Image Processing Toolbox for creating occupancy-map models with
  `build_room_constraint.m`.
- A USB serial connection to the EVB1000 tag.
- The map and calibration files corresponding to the selected environment:
  - `HALL-A_evb1000_map.csv` and hall room model for `main_hall.m`;
  - `DEPT_evb1000_map.csv`, `variance_per_node_dept.csv`, and projective room model for
    `main_dept.m`.
- Real-time firmware running on the tag and responder anchors.

No MATLAB optimization toolbox is required for the WLS initialization: the implementation
uses an explicit Gauss-Newton iteration with pseudo-inverse fallback for ill-conditioned
normal equations.

## Experimental Setup

### Hall Baseline

`main_hall.m` is the baseline configuration used in the report. It tracks with anchor nodes
`50:58`, `61:65`, and `70:77`, loads the hall occupancy model, and applies a unicycle EKF
with process noise on linear speed and yaw rate. A heading pseudo-measurement is inferred
from displacement when the estimated motion is sufficiently large.

### Department and Corridor Configuration

`main_dept.m` targets the Department/corridor layout. It uses node `108`, nodes `113:119`,
and nodes `121:154`. The script adds a corridor-aware heading prior: outside corridors the
heading cue comes from displacement, while inside corridor bounds the heading target is
aligned with the nearest corridor axis. The prior is disabled in the atrium region to avoid
over-constraining open-space motion.

### Common Runtime Assumptions

- Ranging period: `dT = 0.5 s`.
- Fixed tag height in the range model: `z_fixed_m = 1.30 m`.
- Room constraint enabled at initialization and update.
- Room constraint disabled during prediction in the current configuration.
- Projection/rejection threshold: `0.70 m`.
- Measurement covariance: calibrated per-node standard deviation plus a distance-dependent
  term `sigma_i^2(d) = sigma_0,i^2 + k d_i^2`, with `k = 2e-5`.

## Algorithmic Pipeline

1. Open the serial port connected to the mobile tag.
2. Read complete SS-TWR rounds from the textual stream.
3. Deduplicate repeated anchor measurements, keeping the best sample according to LOS flag,
   quality score, and preamble count.
4. Convert anchor short addresses to metric coordinates.
5. Correct distances using the available bias table.
6. Initialize `[px, py, theta, v, omega]` with WLS on the first usable round.
7. Predict the state using the unicycle model.
8. Optionally inject a heading pseudo-measurement.
9. Update the EKF directly from all valid range measurements in the round.
10. Apply the occupancy-map consistency layer:
    - `project`: move the corrected state to nearby free space;
    - `reject`: discard the correction and keep the prediction.
11. Update the real-time visualization and optional GIF recording.

## How to Run

1. Start the EVB1000 tag firmware and make sure the tag is connected to the host through USB.
2. Confirm that the responder anchors are running the firmware generated in
   `real-time-tracking/`.
3. Open MATLAB in this directory:

```matlab
cd Matlab_Sim
```

4. Edit the serial port in the chosen script if necessary:

```matlab
serial_port = "/dev/tty.usbmodem00000000050C1";  % macOS/Linux example
serial_port = "COM4";                            % Windows example
```

5. Run the Hall baseline:

```matlab
main_hall
```

or run the Department/corridor configuration:

```matlab
main_dept
```

The script waits until a usable initialization round is available. During execution the
console reports update modes such as prediction, update, map projection, and map rejection.

## Occupancy-Map Preparation

Occupancy models can be regenerated with:

```matlab
build_room_constraint('zone', 'HALL')
```

or:

```matlab
build_room_constraint('zone', 'DEPT')
```

The builder is interactive: the user selects calibration anchors on the floorplan, draws the
walkable region, and saves a `room_constraint_model_*.mat` file. When using a new machine or
map image, pass an explicit absolute `map_image_path`.

## Results

In the experiments described in the report, WLS initialization provides rapid startup without
requiring a long multi-round bootstrap. The EKF reduces the frame-to-frame jitter of direct
trilateration and maintains continuous tracking in areas with adequate anchor visibility.
The map layer improves physical plausibility near walls by projecting small violations to
free space and rejecting larger inconsistent updates.

The Department/corridor script addresses a harder geometry: anchor visibility is less uniform
and heading estimates can become unstable near narrow passages. The corridor-aware heading
prior improves directional consistency in those structured regions, while remaining inactive
in the atrium.

The current evaluation remains qualitative because no synchronized dense ground-truth
trajectory is included. Future experiments should add ground-truth acquisition, report
absolute trajectory error, and validate the corridor prior across repeated paths.

## File Guide

- `main_hall.m`: Hall baseline runtime tracker.
- `main_dept.m`: Department/corridor runtime tracker with corridor-aware heading prior.
- `serial_reader.m`: lightweight serial monitor for debugging tag output.
- `build_room_constraint.m`: interactive occupancy-map construction.
- `apply_room_constraint.m`: post-update projection/rejection logic.
- `map_gif_recorder.m`: optional recording utility for runtime visualization.
- `map_viz.m`: exploratory visualization of anchor coordinates and simple cluster layouts.
- `*_evb1000_map.csv`: node coordinates and EVB1000 address metadata.
- `variance_per_node_*.csv`: calibrated range dispersion and bias summaries.
