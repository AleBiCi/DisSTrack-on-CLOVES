# Variances Extraction Module

## Overview

The variances extraction module is responsible for the initial characterization of measurement noise variances from individual anchor nodes in the CLOVES testbed. This is a critical preliminary step in the distributed estimation pipeline, as accurate noise covariance estimates are essential for optimal Kalman filter performance.

The module performs empirical variance extraction by deploying firmware on anchor nodes in controlled conditions and analyzing the statistical properties of their range measurements. These variances are subsequently used to initialize and tune the Extended Kalman Filters in the main localization algorithms.

## Data Processing Pipeline

The `auto_variance_v2` script automates the complete variance extraction workflow:

1. **Firmware Deployment** - Compiles and loads UWB ranging firmware onto anchor nodes
2. **Testbed Reservation** - Allocates exclusive access to CLOVES for the measurement campaign
3. **Data Collection** - Nodes perform continuous ranging measurements under controlled conditions
4. **Variance Analysis** - Processes collected range measurements and computes per-node measurement noise variances
5. **Results Generation** - Produces variance estimates formatted for use by EKF algorithms

## Prerequisites

Before running the variance extraction process, ensure the following are in place:

### 1. Firmware Compilation

- Obtain the firmware source code from the `FW/` folder (location to be specified)
- Set up the proper virtual machine environment with the required toolchain (details pending)
- Compile the UWB ranging firmware binaries targeting EVB1000 hardware

### 2. Testbed Access

- Create a reservation on the CLOVES testbed for **at least 2 hours**
- Ensure the reservation covers the full measurement duration including setup and data collection

### 3. Environment Setup

Create a Python virtual environment and install dependencies:

```bash
python -m venv venv
source venv/bin/activate          # On Windows: venv\Scripts\activate

# Install testbed client dependencies
pip install -r <TESTBED_CLIENT_FOLDER>/requirements.txt
```

## Running the Variance Extraction

### Step 1: Configure Testbed Client Credentials

Before submitting jobs, authenticate with the testbed using your access token:

```bash
./<TESTBED_CLIENT_FOLDER>/iot_testbed_client.py --token XXXX saveConfig
```

Replace `XXXX` with your CLOVES testbed API token.

### Step 2: Execute Variance Extraction

Run the `auto_variance_v2` script with appropriate arguments to submit the variance extraction jobs:

```bash
python auto_variance_v2 [ARGUMENTS]
```

After the submission:
1. Wait for the jobs on the CLOVES testbed to complete.
2. Download the job results with:

```bash
python auto_variance_v2 --download
```

3. Compute the individual and aggregate variances by running:

```bash
python compute_variances.py <path_to_log_folder>
```

Where `<path_to_log_folder>` is the folder containing the downloaded log files.

Refer to the script documentation or `auto_variance_v2 --help` for additional arguments and configuration options.

## Output

The module generates variance estimates for each anchor node, typically stored as:
- Per-node measurement noise variances (σ²)
- Confidence intervals or uncertainty bounds
- Statistical summaries of collected range measurements
- Configuration files ready for EKF initialization

These outputs are subsequently used by the MATLAB simulation modules to configure optimal filter gains.
