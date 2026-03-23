# DisSTrack-on-CLOVES

Distributed System for 2D Tracking on CLOVES UniTN IoT Testbed

## Overview

DisSTrack-on-CLOVES is a distributed localization and tracking system designed for the CLOVES IoT testbed at the University of Trento. The system estimates the real-time 2D position of a mobile target using Ultra-Wideband (UWB) ranging measurements from an array of fixed anchor nodes. The implementation combines distributed Extended Kalman Filtering (EKF) with dynamic clustering to achieve scalable and robust multi-agent estimation in real-world indoor environments.

**Key Features:**
- Distributed Extended Kalman Filter (EKF) with dynamic clustering for scalability
- UWB-based ranging using the Single-Sided Two-Way Ranging (SS-TWR) protocol
- Integration with real hardware: EVB1000 boards running Contiki OS and DW1000 UWB transceivers
- MATLAB-based simulation and data processing pipeline
- Support for both offline analysis and real-time tracking scenarios

## Requirements

### MATLAB
- MATLAB R2019a or later

### Python
- Python 3.6+
- Dependencies for `rng_eval.py`:
  - pandas
  - numpy
  - matplotlib (optional, for visualization)

### Hardware (for experimental deployment)
- EVB1000 development boards with DW1000 UWB radios
- Contiki OS compatible toolchain (for building UWB ranging firmware)

## How to Run

<!-- Simulation instructions go here -->

## Project Contents

```
DisSTrack-on-CLOVES/
│
├── README.md                               Project documentation
├── .gitignore                              Git ignore rules
│
├── Matlab_Sim/                             Main simulation and data processing
│   ├── DynamicCluster_and_DistributedKalman.m
│   │                                       Principal EKF implementation with
│   │                                       dynamic clustering for distributed
│   │                                       target tracking
│   ├── DC_IEKF_OR.m                        Alternative iterated EKF variant
│   │                                       with outlier rejection capabilities
│   ├── cloves_to_ekf.m                     Data parser: converts raw UWB
│   │                                       ranging logs to EKF input format
│   ├── map_viz.m                           Testbed topology visualization
│   │                                       and cluster configuration display
│   ├── importfile.m                        Utility function for CSV import
│   │
│   └── DEPT_evb1000_map.csv                Node deployment database with
│                                           addresses, coordinates, and meta
│
├── refs/                                   Reference materials and examples
│   │
│   ├── Lab9.pdf                            Reference documentation
│   ├── disi_povo1_map.png                  CLOVES testbed floor map
│   │
│   └── uwb-rng-radio_example/              UWB ranging protocol reference
│       ├── rng-init.c                      Initiator node (active ranging)
│       ├── rng-resp.c                      Responder node (passive replies)
│       ├── rng-support.c                   Shared UWB radio primitives
│       ├── rng-support.h                   UWB library headers
│       ├── project-conf.h                  Contiki OS radio configuration
│       ├── Makefile                        Build system for UWB firmware
│       │
│       ├── rng_eval.py                     Ranging validation and accuracy
│       │                                   analysis
│       ├── experiment.json                 Experiment configuration and
│       │                                   parameters
│       ├── code_explanation.md             Detailed protocol documentation
│       │
│       └── DEPT_evb1000_map.csv            Reference node coordinates
│
└── variances_extraction/                   Utility directory
    └── bins/                               Compiled binaries output folder
```

### File Descriptions

**Matlab_Sim Module** - Core simulation engine
- `DynamicCluster_and_DistributedKalman.m`: Main EKF-based localization algorithm with dynamic clustering of anchor nodes for scalable multi-agent estimation
- `DC_IEKF_OR.m`: Variant implementation exploring iterated EKF or outlier rejection techniques
- `cloves_to_ekf.m`: Bridge between raw experimental logs and EKF input format
- `map_viz.m`: Visualizes CLOVES testbed topology and cluster organization
- `DEPT_evb1000_map.csv`: Testbed node database with deployment coordinates

**refs/uwb-rng-radio_example** - UWB ranging implementation reference
- `rng-init.c/rng-resp.c`: Contiki OS firmware implementing SS-TWR protocol for distance measurements
- `rng-support.c/.h`: Low-level DW1000 UWB radio API and timestamp handling
- `rng_eval.py`: Validation utility comparing measured vs. ground-truth distances
- `experiment.json`: Experiment scenario definitions and node configurations
- `code_explanation.md`: Comprehensive SS-TWR protocol walkthrough
