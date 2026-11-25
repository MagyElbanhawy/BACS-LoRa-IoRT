# `bacs_scheduler` ROS 2 Package

The `bacs_scheduler` package implements bandwidth-aware constraint scheduling for multi-robot SLAM over low-throughput, duty-cycle-constrained communication channels such as LoRa.

## Package Structure

```
bacs_scheduler/
├── bacs_scheduler/
│   ├── __init__.py
│   ├── bacs.py                     # Core BACS heuristic ranking
│   ├── bacs_plus.py                # BACS+ adaptive geometric gating & spatial diversity
│   ├── fifo.py                     # FIFO baseline scheduler
│   ├── random.py                   # Random baseline scheduler
│   ├── scheduler.py                # Base scheduler interface and priority queue
│   ├── scheduler_node.py           # Standard scheduler ROS 2 node
│   ├── scheduler_node_bacs.py      # BACS/BACS+ specialized ROS 2 node
│   ├── sender_node.py              # Serial LoRa transmitter node with 1% duty-cycle regulation
│   ├── sender_core.py              # Serialization and packet formatting core
│   ├── receiver_node.py            # Serial LoRa receiver node with CRC verification
│   ├── receiver_core.py            # Deserialization and payload parsing core
│   ├── candidate_node.py           # Inter-robot loop closure detection node
│   ├── candidate_generator.py      # Scan-matching and pose graph candidate extraction
│   ├── fusion_node.py              # Multi-robot pose-graph optimization node (GTSAM)
│   ├── fusion_core.py              # Global factor graph management
│   ├── rylr998.py                  # REYAX RYLR998 AT command serial interface
│   ├── lora.py                     # LoRa PHY timing and airtime calculation models
│   ├── lora_serial_bridge.py       # Serial port framing and error handling
│   ├── duty_cycle.py               # ETSI 1% sliding-window duty-cycle budget tracker
│   ├── observability.py            # Fisher information matrix computation
│   ├── place_recognition.py        # Descriptor-based loop closure matching
│   ├── transport.py                # Network packet abstraction and acknowledgement logic
│   └── hardware_session.py         # Hardware trial orchestration and telemetry logging
├── launch/
│   ├── session_bacs.launch.py      # Production BACS+ session launch script
│   ├── session.launch.py           # Configurable comparative policy launch script
│   └── receiver.launch.py          # Base-station receiver and optimizer launch script
├── resource/
│   └── bacs_scheduler              # Ament resource marker
├── package.xml                     # Package metadata and ROS 2 dependencies
├── setup.py                        # Python setuptools build configuration
└── setup.cfg                       # Setuptools installation targets
```

## Supported Scheduling Policies

1. **`BACS_PLUS`**: Evaluates prospective constraints by maximizing the ratio of Fisher information gain $\Delta \mathcal{I}(z)$ to RF transmission airtime $\tau(z)$, penalized by age-of-information decay $e^{-\lambda \Delta t}$, augmented with dynamic geometric gating $\gamma_{\text{gate}}$ and dual-threshold loop closure admission.
2. **`BACS`**: Fisher information per unit airtime with exponential age penalty without adaptive geometric gating.
3. **`FIFO`**: Standard First-In First-Out admission order subject to the transmission airtime budget.
4. **`RANDOM`**: Uniform random constraint selection under the airtime budget.
5. **`LIFO`**: Last-In First-Out prioritizing newest constraints.

## Building and Testing

```bash
# Sourcing ROS 2 Humble
source /opt/ros/humble/setup.bash

# Build package
colcon build --packages-select bacs_scheduler

# Source package overlay
source install/setup.bash

# Run unit tests
colcon test --packages-select bacs_scheduler
```
