This code implements a **Single-Sided Two-Way Ranging (SS-TWR)** protocol for Ultra-Wideband (UWB) distance measurement in a distributed network of low-power anchor nodes based on the EVB1000 development board (which integrates the DW1000 UWB transceiver chip). The system is designed for the CLOVES (likely a research or experimental distributed localization network) project, where anchors collaboratively estimate their relative positions through pairwise distance measurements. These measurements can feed into distributed estimation algorithms (e.g., Kalman filters or optimization-based localization) to build a map of the network without relying on external infrastructure like GPS.

The code is written in C for the Contiki operating system (a lightweight OS for IoT devices), targeting the DW1000 UWB radio for precise ranging. It compiles into two binaries: one for the **initiator** node (which actively starts ranging rounds) and one for **responder** nodes (which react to ranging requests). The binaries are flashed onto specific nodes as defined in `experiment.json` (initiator on node 1, responders on nodes 2, 3, 7, and 36). The network operates in a round-robin fashion, where the initiator cycles through responders to measure distances sequentially.

Below, I'll break down the code's functionality in detail, file by file, including the protocol mechanics, hardware interactions, and how it integrates into the distributed network.

### 1. **Overall Purpose and Protocol Overview**
- **Goal**: Measure the physical distance between pairs of UWB anchor nodes in a distributed network. Distances are computed from round-trip time-of-flight (ToF) measurements of radio signals, leveraging UWB's high precision (sub-centimeter accuracy potential) and resistance to multipath interference.
- **SS-TWR Protocol**: This is a variant of Two-Way Ranging (TWR) where only one side (the responder) embeds timestamps in its response. It avoids clock synchronization between nodes by exchanging timestamps in messages.
  - **Key Idea**: The initiator sends a "ping" (init message), the responder replies with a "pong" (resp message) after a fixed delay. Timestamps are recorded at each node for transmission/reception events. The initiator uses these to compute the ToF, accounting for the responder's delay.
  - **Why SS-TWR?**: Simpler than full TWR (which requires synchronized clocks) and suitable for low-power, distributed systems where nodes can't maintain global time.
  - **Accuracy Factors**: UWB timestamps are in device time units (~15.65 ps resolution). The code handles clock overflows (40-bit timestamps wrap every ~17 seconds) and antenna delays.
- **Network Role**: In CLOVES, this enables distributed localization. Measured distances form a graph where nodes are vertices and distances are edges. Algorithms (possibly implemented in MATLAB files like DynamicCluster_and_DistributedKalman.m in your workspace) use these to estimate positions without a central coordinator.
- **Hardware/Software Stack**:
  - **DW1000 Chip**: Handles UWB transmission/reception, timestamping, and ranging primitives.
  - **Contiki OS**: Manages processes, timers, and networking.
  - **Configuration**: `project-conf.h` sets UWB parameters (e.g., channel 4, 6.8 Mbps data rate) for reliable indoor ranging.
- **Experiment Setup**: `experiment.json` defines a 120-second run on the "DEPT" testbed, flashing binaries to specific nodes. Logs from nodes are parsed by `rng_eval.py` to compute ranging errors against ground-truth positions from DEPT_evb1000_map.csv.

### 2. **Key Files and Their Roles**
#### **rng-init.c (Initiator Code)**
- **Role**: Runs on the initiator node (e.g., node 1). It orchestrates ranging rounds by sending requests to responders in a round-robin cycle.
- **Process Flow**:
  1. **Initialization**: Prints the node's address, initializes the UWB radio (via `radio_init()`), and sets up a timer for ranging intervals (0.5 seconds).
  2. **Ranging Loop**:
     - Increments a sequence number for each round.
     - Selects the next responder from a hardcoded list (`resp_list`).
     - Waits for the interval timer.
     - Prepares an `sstwr_init_msg_t` packet (just a header with source/destination addresses and sequence number).
     - Transmits the init message immediately (`DWT_START_TX_IMMEDIATE`) and expects a response (`DWT_RESPONSE_EXPECTED`), with a 5-second RX timeout.
     - If TX fails, resets the radio and retries.
     - Waits for the response packet.
     - If RX fails or the packet size is wrong, resets and retries.
     - Extracts timestamps from the response message:
       - Initiator's TX timestamp (when init was sent).
       - Initiator's RX timestamp (when resp was received).
       - Responder's RX timestamp (when init was received, embedded in resp).
       - Responder's TX timestamp (when resp was sent, embedded in resp).
     - Computes ToF using SS-TWR formula:
       - `t_one = (resp_rx_ts - init_tx_ts) % DWT_VALUES` (total round-trip time from initiator's perspective).
       - `t_two = (resp_tx_ts - init_rx_ts) % DWT_VALUES` (total round-trip time from responder's perspective).
       - `tof = ((t_one - t_two) / 2.0) * DWT_TIME_UNITS` (average one-way time, accounting for responder delay).
       - Handles potential clock overflows with modulo operations.
     - Converts ToF to distance: `dist = tof * SPEED_OF_LIGHT`, then to millimeters.
     - Prints the result (e.g., "RANGING OK [11:0c->19:15] 169 mm").
     - If addresses don't match expectations, logs a failure and continues.
- **Key Features**:
  - **Error Handling**: Resets radio on TX/RX failures to recover from transient issues.
  - **Round-Robin**: Cycles through responders to avoid overloading any one node.
  - **Low Power**: Uses timers and blocking waits to minimize energy use.
- **Integration**: Outputs logs that `rng_eval.py` parses to validate against ground-truth distances.

#### **rng-resp.c (Responder Code)**
- **Role**: Runs on responder nodes (e.g., nodes 2, 3, 7, 36). They passively listen for init messages and respond with timestamped replies.
- **Process Flow**:
  1. **Initialization**: Prints the node's address and initializes the UWB radio.
  2. **Listening Loop**:
     - Enables RX immediately with no timeout.
     - Waits for an init message.
     - If RX fails or size is wrong, resets and retries.
     - Extracts source/destination from the packet header.
     - If the destination matches this node:
       - Records the RX timestamp (when init was received).
       - Computes a delayed TX time: `resp_tx_ts = (init_rx_ts + UWB_RESP_DELAY) % DWT_VALUES` (adds a 500 µs delay to simulate processing/avoid collisions).
       - Prepares an `sstwr_resp_msg_t` packet (header plus embedded timestamps).
       - Predicts the actual TX timestamp (accounting for antenna delays and granularity).
       - Embeds timestamps in the response: init RX time and predicted resp TX time.
       - Schedules a delayed transmission (`DWT_START_TX_DELAYED`) at the computed time.
       - Waits for TX confirmation and prints success/failure.
     - If destination doesn't match, ignores the packet.
- **Key Features**:
  - **Delay Mechanism**: The fixed 500 µs delay ensures the responder doesn't reply instantly, allowing accurate timestamping.
  - **Timestamp Embedding**: Uses `resp_msg_set_timestamp()` to pack 64-bit timestamps into 8-byte fields.
  - **Passive Operation**: Responders only TX when addressed, conserving power.

#### **rng-support.c and rng-support.h (Shared Support Library)**
- **Purpose**: Provides low-level UWB primitives, timestamp management, and packet handling. Used by both initiator and responder.
- **Key Functions**:
  - **Timestamping**:
    - `get_rx_timestamp()` / `get_tx_timestamp()`: Reads 40-bit timestamps from DW1000 registers.
    - `predict_tx_timestamp()`: Adjusts scheduled TX time for antenna delays (~16,455 device units).
    - `resp_msg_set_timestamp()` / `resp_msg_get_timestamp()`: Packs/unpacks timestamps into message fields (little-endian byte order).
  - **Radio Control**:
    - `radio_init()`: Configures DW1000 (disables interrupts, sets antenna delays, frame filtering).
    - `start_tx()`: Handles TX with modes (immediate/delayed), RX expectations, and timeouts. Includes errata workarounds (e.g., TX-1 for clock settings).
    - `start_rx()` / `wait_rx()` / `read_rx_data()`: Manages RX, waiting for packets, and reading data.
    - `radio_reset()`: Clears errors and resets transceiver after failures.
  - **Packet Formatting**:
    - `fill_ieee_hdr()`: Sets IEEE 802.15.4 headers (source, destination, sequence number, PAN ID).
- **Constants**:
  - `DWT_VALUES`: 2^40 (timestamp range).
  - `SPEED_OF_LIGHT`: 299,702,547 m/s (in air).
  - Antenna delay, CRC length, etc., tuned for DW1000.

#### **project-conf.h (Configuration)**
- Defines UWB settings for reliable ranging:
  - Channel 4, 64 MHz PRF, 128-symbol preamble, 6.8 Mbps rate.
  - Optimized for indoor environments (reduces interference).

#### **experiment.json (Deployment Config)**
- Specifies flashing: `rng-init.bin` to node 1, `rng-resp.bin` to nodes 2/3/7/36.
- Runs for 120 seconds on the DEPT testbed.

#### **rng_eval.py (Evaluation Script)**
- Parses node logs for ranging results (e.g., "RANGING OK [init->resp] dist mm").
- Maps node addresses to IDs/coordinates using DEPT_evb1000_map.csv.
- Computes errors: measured distance vs. Euclidean distance from ground truth.
- Outputs per-pair errors (e.g., "Ranging error [1->2] -5 mm").

#### **Makefile (Build System)**
- Compiles Contiki projects for EVB1000 hardware, producing the binaries.

### 3. **How It Fits into CLOVES Distributed Network**
- **Distributed Nature**: No central server—nodes self-organize via ranging. The initiator drives measurements, but responders are symmetric.
- **Scalability**: Round-robin limits concurrent TX, avoiding collisions in dense networks.
- **Integration with Your Workspace**: Measured distances (logged as "RANGING OK") can feed into MATLAB scripts (e.g., loc_track_MASTER.m or DynamicCluster_and_DistributedKalman.m) for EKF-based localization. DEPT_evb1000_map.csv provides ground truth for validation.
- **Challenges Addressed**: Clock drift (handled via SS-TWR), multipath (UWB robustness), power constraints (low-duty-cycle operation).
- **Potential Extensions**: Add more nodes, implement distributed consensus for position estimation, or integrate with motion tracking (e.g., localization_movingtag_EKF.m).

This code is production-ready for flashing onto EVB1000 nodes, enabling autonomous distance-based localization in your distributed network. If you need help modifying it (e.g., adding more nodes or integrating with MATLAB), let me know!