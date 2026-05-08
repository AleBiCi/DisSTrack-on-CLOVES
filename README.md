# DisSTrack-on-CLOVES

Real-time UWB localization and tracking on the University of Trento CLOVES testbed.

## Overview

DisSTrack-on-CLOVES is an experimental indoor localization pipeline for tracking a mobile
Ultra-Wideband (UWB) tag in the CLOVES infrastructure. The project combines embedded
Single-Sided Two-Way Ranging (SS-TWR), calibration-based range correction, and MATLAB
state estimation in order to obtain a physically plausible two-dimensional trajectory from
noisy range-only measurements.

The implemented system follows a layered architecture. EVB1000 nodes perform the ranging
rounds and apply an initial firmware-level quality filter. A host computer connected to the
mobile tag reconstructs complete rounds from the serial stream, maps UWB short addresses
to known anchor coordinates, compensates calibrated biases, and runs a range-only Extended
Kalman Filter (EKF). The estimator uses a unicycle state model, weighted least-squares
initialization, adaptive measurement covariance, and occupancy-map consistency checks.

Although the repository title emphasizes distributed tracking, the current implementation is
best described as distributed ranging with centralized real-time estimation: anchors and tag
cooperate to produce round-based measurements, while the EKF runs on the host.

## Main Contributions

- Real-time SS-TWR ranging firmware for EVB1000/DW1000 nodes.
- Round-based multi-anchor acquisition with scheduled responder slots.
- Per-anchor calibration tables for range bias and variance compensation.
- Weighted single-round initialization for rapid online startup.
- Range-only unicycle EKF using all valid anchors available in each round.
- Map projection/rejection layer to avoid physically implausible wall crossings.
- Separate Hall and Department/corridor configurations for different CLOVES layouts.

## Repository Structure

```text
.
|-- README.md
|-- CLOVES_client/
|   |-- iot_testbed_client.py
|   `-- requirements.txt
|-- Matlab_Sim/
|   |-- README_main.md
|   |-- main_hall.m
|   |-- main_dept.m
|   |-- build_room_constraint.m
|   |-- apply_room_constraint.m
|   |-- HALL-A_evb1000_map.csv
|   |-- DEPT_evb1000_map.csv
|   |-- variance_per_node_dept.csv
|   `-- room_constraint_model_*.mat
|-- real-time-tracking/
|   |-- README.md
|   |-- rng-init-all.c
|   |-- rng-resp.c
|   |-- generate_and_build_all.py
|   |-- anchor_table.h
|   |-- experiment.json
|   |-- Makefile
|   `-- cloves-client/
|-- variances_extraction/
|-- variances_extraction_HALL_A/
`-- refs/
```

## Requirements

### Hardware and Testbed Access

- Decawave/Qorvo EVB1000 boards equipped with DW1000 UWB radios.
- Access to the University of Trento CLOVES IoT testbed.
- One EVB1000 node acting as the mobile tag and connected to the host via USB serial.
- Multiple EVB1000 anchor nodes deployed at known metric coordinates.

### Firmware Toolchain

- Contiki/UWB build environment for the EVB1000 target.
- `make` and the compiler toolchain expected by `Makefile.uwb`.
- A valid `UWB_CONTIKI` path when the firmware folder is not located inside the Contiki tree.

### MATLAB

- MATLAB with `serialport` support.
- Image Processing Toolbox is recommended for occupancy-map construction and validation.
- The scripts were developed for recent MATLAB releases; if an older release is used, minor
  graphics-function substitutions may be required.

### Python

Python is used for firmware support generation and CLOVES job submission.

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r CLOVES_client/requirements.txt
pip install -r real-time-tracking/cloves-client/requirements.txt
```

The requirements mainly cover the CLOVES client and data-handling utilities:

- `requests`
- `certifi`
- `urllib3`
- `numpy`
- `pandas`

## Experimental Setup

The project report describes two runtime configurations.

### Hall Baseline

- MATLAB entry point: `Matlab_Sim/main_hall.m`.
- Anchor set: nodes `50:58`, `61:65`, and `70:77`.
- Map model: `room_constraint_model_hall.mat`.
- Purpose: baseline real-time tracking in a better-covered open indoor area.

### Department/Corridor Configuration

- MATLAB entry point: `Matlab_Sim/main_dept.m`.
- Anchor set: node `108`, nodes `113:119`, and nodes `121:154`.
- Map model: `room_constraint_model_projective.mat`.
- Purpose: evaluation in a narrower corridor-like environment with weaker and less symmetric
  anchor visibility.

In both configurations the UWB tag height is fixed in software at `z_fixed_m = 1.30 m`, and
the ranging period is `dT = 0.5 s`. Calibration logs are used to estimate per-node dispersion
and range bias. These statistics define the nominal range covariance used during weighted
initialization and EKF updates.

## How to Run

### 1. Build the Real-Time Firmware

From the firmware directory:

```bash
cd real-time-tracking
python3 generate_and_build_all.py
```

The generator rebuilds support files from the deployment map and variance table, compiles
the tag and responder firmware, and writes binaries under `real-time-tracking/bins/`.

If the Contiki tree is not the parent of this repository, set `UWB_CONTIKI` before building:

```bash
UWB_CONTIKI=/path/to/contiki-uwb python3 generate_and_build_all.py
```

### 2. Deploy the Responder Firmware

Use the CLOVES client and `real-time-tracking/experiment.json` to schedule the responder
binary on the selected anchor nodes:

```bash
cd real-time-tracking
python3 cloves-client/iot_testbed_client.py validate experiment.json
python3 cloves-client/iot_testbed_client.py schedule --asap experiment.json
```

The job file must reference the actual responder binary path used for the run. If the
generated binary is under `bins/`, update `bin_file` accordingly or place the binary next to
the job file before submission.

### 3. Flash or Start the Mobile Tag

Flash `bins/rng-init-all.bin` on the EVB1000 tag. The tag must remain connected by USB to
the host computer because MATLAB reads its serial output.

### 4. Run the MATLAB Tracker

Open MATLAB in `Matlab_Sim/`, verify the serial port configured in the selected script, and
run one of:

```matlab
main_hall
```

or:

```matlab
main_dept
```

The MATLAB tracker waits for a usable ranging round, initializes the state with weighted
trilateration, and then enters the real-time EKF loop. The figure shows anchors, current
estimate, trajectory history, heading, covariance ellipse, and the anchors used in each
round.

## Results

The current evidence is based on hardware runs, runtime logs, map-consistency events, and
visual inspection of the estimated trajectories. A synchronized dense ground-truth trajectory
is not packaged with the repository, therefore the reported evaluation is qualitative rather
than an absolute error benchmark.

In static tests, single-round weighted initialization generally places the tag in the correct
region when the first usable round has acceptable anchor geometry. During walking trials, the
EKF smooths the jitter observed in direct range-based localization and maintains continuous
tracking in well-covered areas. In weakly covered corridor regions, covariance weighting and
prediction fallback reduce abrupt corrections, while map projection or rejection prevents
implausible transitions through walls.

The main limitations are the dependence on first-round geometry, the absence of bundled
ground truth for RMSE curves, the residual sensitivity to long NLOS phases, and the need to
tune occupancy-map thresholds for each environment.

## Notes on Reproducibility

- Keep the map CSV, firmware anchor table, and MATLAB `tracking_node_ids` synchronized.
- Recompute variance and bias tables when the physical anchor layout changes.
- Verify the serial port before running MATLAB.
- Check that the job JSON points to the responder binary actually being submitted.
- Treat the Hall and Department scripts as separate experimental configurations rather than
  interchangeable entry points.

## Reference Context

This repository accompanies the final project report "Real-time distributed localization and
tracking on CLOVES testbed" for the Intelligent Distributed Systems course at the University
of Trento, academic year 2025/26.
