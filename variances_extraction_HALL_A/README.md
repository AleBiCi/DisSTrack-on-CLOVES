# Variance Extraction for the HALL-A Configuration

This directory contains the HALL-A calibration workflow used to estimate UWB ranging
dispersion and bias for the Hall baseline experiments. The resulting variance table is used
by the MATLAB tracker to assign measurement covariance and to compensate systematic range
offsets when running the Hall configuration.

## Overview

The Hall baseline is easier than the Department corridor in terms of geometry, but it still
requires empirical calibration. UWB measurements are affected by per-node dispersion,
multipath, antenna delays, and small systematic offsets. This module estimates those effects
from repeated SS-TWR measurements collected directly on the CLOVES HALL-A island.

The workflow is the same as in the Department calibration: each selected node is compiled as
a dedicated initiator binary, ranges against its nearest responders, and produces serial logs.
The logs are later parsed into pair-level statistics and pooled node-level noise estimates.

## Requirements

- Access to the CLOVES HALL-A island and valid API credentials.
- EVB1000/DW1000 firmware build environment compatible with `Makefile.uwb`.
- Python 3 with:
  - `numpy`
  - `pandas`
  - CLOVES client dependencies from `FW/cloves-client/requirements.txt`.
- `HALL-A_evb1000_map.csv` with node IDs, short UWB addresses, and metric coordinates.
- Per-node initiator binaries in `bins/` and a common `rng-resp.bin` responder binary.

Recommended Python setup:

```bash
cd variances_extraction_HALL_A
python3 -m venv venv
source venv/bin/activate
pip install numpy pandas
pip install -r FW/cloves-client/requirements.txt
```

## Experimental Setup

The active HALL-A calibration set is defined in `auto_variance_v2.py` and
`FW/generate_and_build_all.py`:

```text
70, 71, 72, 73, 74, 75, 76, 77
```

For each initiator node, the automation selects the five nearest responders according to the
metric coordinates in `HALL-A_evb1000_map.csv`. Each job then deploys:

- one `rng-init-nodeNNN.bin` binary to the initiator;
- one `rng-resp.bin` binary to the selected responder nodes.

The default job duration is `120 s`, and downloaded logs are grouped under
`logs_varianza_v2/`.

## How to Run

### 1. Build Per-Node Calibration Firmware

Build the firmware from the HALL-A firmware directory:

```bash
cd variances_extraction_HALL_A/FW
python3 generate_and_build_all.py
```

Expected generated files:

```text
bins/rng-init-node070.bin
bins/rng-init-node071.bin
...
bins/rng-init-node077.bin
bins/rng-resp.bin
```

If the firmware toolchain is outside the expected relative path, export `UWB_CONTIKI` before
running the build.

### 2. Check the Submission Plan

From `variances_extraction_HALL_A/`, run a dry run:

```bash
python3 auto_variance_v2.py --dry-run
```

This prints the initiator-responder assignments without submitting jobs.

### 3. Submit Calibration Jobs

Submit the full HALL-A calibration campaign:

```bash
python3 auto_variance_v2.py
```

Optional controls:

```bash
python3 auto_variance_v2.py --start-from 3
python3 auto_variance_v2.py --duration 180
```

The submitted job IDs are written to:

```text
submitted_jobs_v2.csv
```

### 4. Download Completed Logs

After the jobs complete on CLOVES, download the logs:

```bash
python3 auto_variance_v2.py --download
```

The script stores one subdirectory per initiator node under:

```text
logs_varianza_v2/
```

### 5. Compute Hall Variance Tables

Aggregate the logs:

```bash
python3 compute_variances.py --logs-dir logs_varianza_v2 --csv HALL-A_evb1000_map.csv
```

This produces the Hall calibration tables in the current directory.

## Outputs

- `variance_per_pair.csv`: statistics for each measured initiator-responder pair, including
  mean distance, standard deviation, variance, geometric reference distance, and bias.
- `variance_per_node.csv`: pooled per-initiator statistics used by the EKF, especially
  `pooled_std_m` and `mean_bias_mm`.
- `submitted_jobs_v2.csv`: bookkeeping table connecting node IDs to CLOVES job IDs.
- `logs_varianza_v2/`: downloaded raw serial logs used for the computation.

The Hall MATLAB runtime should use the Hall-specific output table, not the Department table,
because the active nodes, geometry, and radio conditions are different.

## Results and Interpretation

The expected outcome is a compact calibration table for nodes `70` through `77`. In the full
tracking pipeline, these values improve the Hall baseline by assigning more realistic
measurement covariance to each anchor and by correcting repeatable mean range offsets. The
benefit is mainly visible as smoother EKF updates and reduced sensitivity to anchors whose
range samples are consistently more dispersed.

The calibration should be repeated whenever the Hall anchor layout changes, the firmware
timing is modified, or the physical environment changes enough to alter the LOS/NLOS
statistics.

## File Guide

- `auto_variance_v2.py`: submits and downloads the HALL-A calibration campaign.
- `compute_variances.py`: computes pair-level and pooled node-level statistics from logs.
- `HALL-A_evb1000_map.csv`: HALL-A node map used for address mapping and geometric distance
  calculation.
- `experiment.json`: example CLOVES job configuration retained for manual checks.
- `FW/generate_and_build_all.py`: builds one initiator binary per Hall node.
- `FW/bins/`: generated calibration binaries.
- `FW/variance_per_pair.csv` and `FW/variance_per_node.csv`: previously generated reference
  outputs from a Hall calibration run.
- `FW/cloves-client/`: CLOVES command-line client and certificate.

## Reproducibility Notes

- Keep `VALID_NODES` consistent between `auto_variance_v2.py` and
  `FW/generate_and_build_all.py`.
- Keep `HALL-A_evb1000_map.csv` synchronized with the physical testbed coordinates.
- Use Hall-specific variance outputs with `main_hall.m`; do not mix them with DEPT/corridor
  calibration tables.
- Inspect pair-level bias and variance before accepting a table, since a single problematic
  node can distort the pooled covariance used by the EKF.
