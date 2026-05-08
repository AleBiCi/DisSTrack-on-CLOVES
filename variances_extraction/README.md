# Variance Extraction for the DEPT Configuration

This directory contains the calibration workflow used to estimate empirical UWB ranging
dispersion and bias for the Department configuration of the CLOVES testbed. The generated
statistics are consumed by the MATLAB real-time tracker as per-node measurement covariance
and bias-correction inputs.

## Overview

Indoor UWB ranges are affected by hardware-dependent bias, multipath, non-line-of-sight
conditions, and geometry-dependent variance. The EKF described in the project report should
therefore not rely on a single arbitrary range-noise value. Instead, this module performs a
controlled calibration campaign in which each selected EVB1000 node acts as an initiator and
ranges against its nearest neighbors.

For each initiator-responder pair, the analysis script computes sample count, mean range,
standard deviation, variance, approximate geometric ground-truth distance, and mean bias.
The pair-level statistics are then pooled by initiator node to produce the `pooled_std_m`
value used by the EKF measurement covariance model and the `mean_bias_mm` value used for
range compensation.

## Requirements

- Access to the CLOVES testbed and valid API credentials.
- EVB1000/DW1000 firmware build environment compatible with `Makefile.uwb`.
- Python 3 with:
  - `numpy`
  - `pandas`
  - CLOVES client dependencies from `FW/cloves-client/requirements.txt`, when using the
    client copy inside `FW/`, or from the local client path used for the campaign.
- `DEPT_evb1000_map.csv` with valid node identifiers, UWB addresses, and metric coordinates.
- Generated initiator binaries named `rng-init-nodeNNN.bin` and a responder binary
  `rng-resp.bin`.

Recommended Python setup:

```bash
cd variances_extraction
python3 -m venv venv
source venv/bin/activate
pip install numpy pandas
pip install -r FW/cloves-client/requirements.txt
```

## Experimental Setup

The DEPT calibration uses the node set encoded in `auto_variance_v2.py` and
`FW/generate_and_build_all.py`. The valid nodes include the Department anchors used by the
real-time corridor experiments, in particular node `108`, nodes `113:119`, and nodes
`121:154`.

The campaign is performed as repeated local ranging jobs:

1. one node is flashed as the SS-TWR initiator;
2. its five nearest neighboring nodes are flashed as responders;
3. the initiator cycles through the selected responders and prints `RANGING OK` samples;
4. CLOVES stores the serial logs for the job;
5. the analysis script aggregates all downloaded logs.

The default job duration is `120 s`. This value is a practical compromise: it is long enough
to gather a useful number of samples per pair while keeping the full multi-node calibration
campaign manageable on the shared testbed.

## How to Run

### 1. Build Per-Node Calibration Firmware

Build the firmware inside the `FW/` directory:

```bash
cd variances_extraction/FW
python3 generate_and_build_all.py
```

The generator creates one initiator binary per valid node:

```text
bins/rng-init-nodeNNN.bin
```

and also copies:

```text
bins/rng-resp.bin
```

If the Contiki/UWB tree is not the expected parent directory, export the appropriate
`UWB_CONTIKI` path before building.

### 2. Prepare the Calibration Directory

From `variances_extraction/`, make sure the binaries expected by `auto_variance_v2.py` are
available in `bins/`, and that `rng-resp.bin` is available at the working-directory level or
adapt the script paths accordingly.

You can inspect the campaign without submitting jobs:

```bash
python3 auto_variance_v2.py --dry-run
```

### 3. Submit CLOVES Calibration Jobs

Submit the jobs:

```bash
python3 auto_variance_v2.py
```

Useful options:

```bash
python3 auto_variance_v2.py --start-from 10
python3 auto_variance_v2.py --duration 180
```

The script writes the submitted job identifiers to:

```text
submitted_jobs_v2.csv
```

### 4. Download Completed Logs

After the CLOVES jobs finish, download the results:

```bash
python3 auto_variance_v2.py --download
```

Downloaded logs are stored by default under:

```text
logs_varianza_v2/
```

### 5. Compute Variance and Bias Tables

Run the aggregation step:

```bash
python3 compute_variances.py --logs-dir logs_varianza_v2 --csv DEPT_evb1000_map.csv
```

The default paths in `compute_variances.py` can also be used when the logs and map file are
in their standard locations.

## Outputs

The analysis produces two CSV files in the current directory:

- `variance_per_pair.csv`: pair-level statistics for each initiator-responder combination,
  including sample count, mean range, standard deviation, variance, estimated true distance,
  and mean bias.
- `variance_per_node.csv`: pooled node-level statistics, including `pooled_std_m` and
  `mean_bias_mm`.

The MATLAB tracker uses these quantities as follows:

- `pooled_std_m` defines the nominal range-noise scale for a node;
- `mean_bias_mm` is converted to meters and subtracted from runtime ranges when accepted by
  the tracker configuration;
- missing calibration entries fall back to the default noise value defined in the MATLAB
  script.

## Results and Interpretation

The calibration stage does not produce trajectory accuracy metrics; its purpose is to
characterize the measurement process before tracking. A useful calibration run should produce
multiple valid pairs per node and a stable pooled standard deviation. Large mean bias values
or very high pair variances should be inspected because they may indicate persistent NLOS,
poor geometry, wrong map coordinates, or a problematic node.

In the complete tracking pipeline, these tables improve the EKF by reducing overconfidence in
noisy anchors and compensating repeatable range offsets. They are especially important in the
Department/corridor setup, where anchor visibility is non-uniform and long NLOS phases can
otherwise dominate the range update.

## File Guide

- `auto_variance_v2.py`: submits one calibration job per initiator node and downloads logs.
- `compute_variances.py`: parses `RANGING OK` logs and produces pair-level and node-level
  calibration tables.
- `DEPT_evb1000_map.csv`: node map used to associate UWB addresses with node IDs and metric
  coordinates.
- `FW/generate_and_build_all.py`: generates per-node initiator firmware and the common
  responder binary.
- `FW/rng-resp.c`: responder firmware used during calibration.
- `FW/rng-support.c`, `FW/rng-support.h`: shared DW1000/SS-TWR support code.
- `FW/cloves-client/`: CLOVES testbed command-line client used for job submission.

## Reproducibility Notes

- Keep the map CSV synchronized with the physical CLOVES deployment.
- Rebuild the calibration binaries after changing the valid-node set or nearest-neighbor
  policy.
- Recompute the variance tables after moving anchors, changing firmware timing, or modifying
  radio parameters.
- Copy or reference the resulting `variance_per_node.csv` from the MATLAB configuration that
  uses the same anchor layout.
