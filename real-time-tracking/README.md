# Real-Time Tracking Firmware

This directory contains the EVB1000 firmware and deployment utilities used for the real-time
CLOVES UWB tracking experiments. The firmware implements a round-based SS-TWR protocol:
one mobile tag broadcasts an initiation packet, anchors reply in scheduled slots, and the tag
computes ranges and emits a structured serial stream for MATLAB.

## Scope

The firmware layer provides distributed range acquisition. It does not run the EKF; it
supplies calibrated, round-consistent range observations to the MATLAB tracker. The two main
firmware roles are:

- `rng-init-all.c`: mobile tag / initiator firmware;
- `rng-resp.c`: fixed anchor / responder firmware.

The support generator `generate_and_build_all.py` rebuilds anchor-slot metadata and shared
radio support files from the current deployment map before compiling binaries.

## Requirements

- Decawave/Qorvo EVB1000 nodes with DW1000 UWB radios.
- Contiki/UWB toolchain compatible with `Makefile.uwb`.
- `make` and the cross-compiler required by the EVB1000 target.
- Python 3 for the generator script.
- CLOVES testbed credentials and access for remote responder deployment.
- Python dependencies for the bundled testbed client:

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r cloves-client/requirements.txt
```

When the firmware directory is not inside the Contiki/UWB tree, export `UWB_CONTIKI` before
building:

```bash
export UWB_CONTIKI=/path/to/contiki-uwb
```

## Experimental Setup

The current real-time deployment targets the Department layout used in the project report.
The active tracking anchors are node `108`, nodes `113:119`, and nodes `121:154`. Their UWB
short addresses and responder slots are generated into `anchor_table.h`.

The protocol is round-based:

1. The tag broadcasts an `INIT` frame.
2. Each anchor receives the `INIT` frame and schedules one delayed `RESP` frame in its slot.
3. The tag listens for the full response window.
4. For each valid response, the tag computes SS-TWR time of flight and converts it to range.
5. The tag reads DW1000 diagnostics, applies quality checks, deduplicates repeated anchors,
   and prints one serial stream per round.

The timing constants currently generated are:

- `ANCHOR_STEP_UUS = 2000`
- `PER_ANCHOR_TIMEOUT_UUS = 6000`
- `WINDOW_MARGIN_UUS = 3000`

The MATLAB runtime assumes a round period of `0.5 s`, so firmware timing and MATLAB sampling
should be kept consistent.

## Firmware Filtering

The tag applies an initial quality layer before forwarding data to MATLAB:

- physically implausible distances are rejected;
- slightly negative numerical ranges are clamped to zero;
- responses are labelled as LOS or NLOS using first-path and preamble diagnostics;
- duplicate responses from the same anchor are reduced to the best observation.

The duplicate-selection priority is:

1. LOS measurements over NLOS measurements;
2. higher quality score;
3. higher preamble count.

This filtering is intentionally conservative: MATLAB still performs calibration-based
weighting and map-aware state validation, but clearly corrupted radio observations should be
removed as early as possible.

## Serial Output Format

Each accepted measurement is printed as:

```text
RANGING MEAS [round] [tag->anchor] distance_mm QUAL qual_x10 pream FLAG LOS/NLOS
```

Example:

```text
RANGING MEAS [211] [54:33->5b:2a] 8276 mm QUAL -41 118 FLAG LOS
```

At the end of each round, the tag prints a summary:

```text
[211] round: 8 meas | 5 LOS | 3 NLOS | 24 timeout | 8 dup
```

MATLAB uses these lines to reconstruct complete rounds and associate each distance with the
correct anchor coordinates.

## How to Run

### 1. Generate Support Files and Build Binaries

Run the generator from this directory:

```bash
python3 generate_and_build_all.py
```

The script:

- reads `DEPT_evb1000_map.csv`;
- reads `variance_per_node.csv` when available;
- selects the configured tracking anchors;
- writes `anchor_table.h`;
- writes `rng-support.h` and `rng-support.c`;
- builds `rng-init-all` and `rng-resp`;
- copies the resulting binaries into `bins/`.

Expected outputs include:

```text
bins/rng-init-all.bin
bins/rng-resp.bin
```

### 2. Deploy the Anchor Responders

Validate the CLOVES job file:

```bash
python3 cloves-client/iot_testbed_client.py validate experiment.json
```

Schedule it as soon as possible:

```bash
python3 cloves-client/iot_testbed_client.py schedule --asap experiment.json
```

Before scheduling, ensure that the `bin_file` entry in `experiment.json` points to the
responder binary location used for the run. The provided JSON lists the Department anchor set
and may need to be adapted when testing a reduced subset or a different layout.

### 3. Flash the Mobile Tag

Flash the initiator binary on the mobile EVB1000 tag:

```text
bins/rng-init-all.bin
```

Keep this tag connected to the host computer via USB. The MATLAB tracker reads only the tag
serial stream; anchors do not communicate directly with MATLAB.

### 4. Start Host-Side Tracking

After the responders are active and the tag is connected, run the corresponding MATLAB
script from `Matlab_Sim/`:

```matlab
main_dept
```

or, for the Hall baseline:

```matlab
main_hall
```

## Results

The firmware layer successfully provides grouped multi-anchor rounds for the MATLAB EKF. In
well-covered regions, several anchors are available per round and the host estimator receives
enough information for continuous correction. In weaker corridor regions, timeouts and NLOS
labels become more frequent; the serial summaries make this degradation visible and allow the
host tracker to fall back to prediction or map-based rejection when required.

The main practical benefit of the firmware design is that the tag exports a compact,
round-consistent stream rather than independent asynchronous samples. This reduces ambiguity
in the estimator and makes each EKF update correspond to a coherent acquisition instant.

Current limitations include dependence on correct slot generation, sensitivity to anchor
visibility, and the need to keep the firmware anchor set synchronized with MATLAB maps and
variance tables.

## File Guide

- `rng-init-all.c`: initiator/tag firmware; computes ranges and writes serial output.
- `rng-resp.c`: responder/anchor firmware; replies in scheduled slots with timestamps and
  radio diagnostics.
- `generate_and_build_all.py`: support-file generation and build orchestration.
- `anchor_table.h`: generated anchor address and slot table.
- `rng-support.c`, `rng-support.h`: generated DW1000 helper routines and packet structures.
- `project-conf.h`: Contiki/DW1000 radio configuration.
- `Makefile`: EVB1000 build target configuration.
- `experiment.json`: CLOVES job file for deploying responder firmware to anchors.
- `experiment_debug_stable.json`: reduced responder set used for debugging.
- `variance_per_node.csv`: calibration statistics used by the generator and MATLAB pipeline.
- `cloves-client/`: local copy of the CLOVES testbed command-line client.
