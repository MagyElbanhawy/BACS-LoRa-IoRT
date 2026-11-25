# Hardware experimental platform

This folder documents the physical component of the BACS research project: a two-robot ROS 2 experimental platform with LoRa inter-robot communication. It complements the simulation-based evaluation in `bacs_sim/` and `runs/v15/`. The hardware component measures **communication and scheduling behaviour** (packet airtime, delivery, clock agreement and queue backlog). Map-alignment accuracy in this project is a **simulation** result; no hardware map-accuracy result is reported.

---

## 1. Two-robot experimental architecture

```
 Robot endpoint LIMO-01 (transmitter)                 Robot endpoint LIMO-02 (receiver)
 ┌──────────────────────────────────┐                 ┌───────────────────────────────────┐
 │ ARM64 embedded computer          │                 │ Intel Core i9-10885H, Ubuntu 22.04 │
 │ Ubuntu 18.04, Python 3.6          │                 │ ROS 2 Humble                       │
 │ Scheduler (FIFO in HWS-BACS-001)  │                 │ Receiver node, logging             │
 │        │ USB serial (115200 baud) │                 │        ▲ USB serial (115200 baud)  │
 │        ▼                          │   868 MHz LoRa  │        │                           │
 │ MKR WAN 1310 (SX1276) ───────────────────────────────▶ MKR WAN 1310 (SX1276)            │
 └──────────────────────────────────┘                 └───────────────────────────────────┘
```

- **Robot platforms.** The two endpoints are labelled LIMO-01 and LIMO-02 in the logs. The robot platforms, including the AgileX LIMO, are shown in the photographs and in Videos V4, V5 and V7. In session HWS-BACS-001 both endpoints were stationary and close together.
- **ROS 2 integration.** The receiver runs ROS 2 Humble and records received constraints; the ROS 2 scheduler, sender and receiver packages are in `ros2_ws/src/bacs_scheduler/`.
- **Radios.** Two Arduino MKR WAN 1310 boards (Murata CMWX1ZZABZ module, Semtech SX1276). Firmware: `hardware/firmware/MR/MR.ino` (with `MR_portable` and `MR_debug` variants). The firmware logs every TX and RX event with the board uptime over USB serial.

## 2. LoRa communication configuration

| Parameter | Value |
|---|---|
| Frequency | 868 MHz (EU868) |
| Spreading factor / bandwidth / coding rate | SF7 / 125 kHz / 4/5 |
| Preamble / header / CRC | 8 symbols / explicit / on |
| Transmit power | 14 dBm |
| Payload | 52 bytes (robot and node identifiers, SE(2) transform, compressed information matrix, timestamp, sequence number, predicted trust and information, pair count and flags) |
| Time on air (model) | 102.7 ms |
| Duty-cycle enforcement | In software on the transmitter: one packet every 11.0 s (0.94%) |

## 3. Experimental workflow (session HWS-BACS-001)

1. Start the receiver (ROS 2 node and serial logger) and record host clock status (`chronyc tracking`).
2. Start the transmitter: a synthetic candidate stream generates one inter-robot constraint every 2 s (300 in 600 s); the scheduler (FIFO) hands one packet to the radio every 11.0 s.
3. Both boards log every event with their own uptime clock; both hosts log wall-clock and monotonic timestamps.
4. After the session, write manifests, clock records and SHA-256 checksums.
5. Segment the receiver log (segmentation table in the Supplementary Material) and compute the metrics directly from the raw logs.

The capture scripts are `scripts/record_session.sh`, `scripts/record_receiver.sh` and `scripts/record_bacs_sessions.sh`.

## 4. Measured communication results (hardware)

| Quantity (hardware) | Value | Evidence file |
|---|---|---|
| Candidates generated / packets transmitted | 300 / 54 (FIFO, 11.0 s cadence) | `transmitter/scheduler_decisions.csv`, `transmitter/serial_log.csv` |
| Packets received | 54 of 54 unique sequence numbers; all payloads byte-identical to the transmitted ones | `receiver/receiver_serial_log.csv` |
| Receiver double reports (not counted) | 8, each 20–21 ms after the first report | `receiver/receiver_serial_log.csv` |
| Other receiver rows (not counted) | 11 rows from test transmissions before the session; 1 noise frame (−118 dBm) about 33 minutes after it | `receiver/receiver_serial_log.csv` |
| On-air time per packet (board-reported) | 103–104 ms, mean 103.6 ms (model 102.7 ms) | `transmitter/serial_log.csv` |
| Hand-off to transmit-done | 110–121 ms, median 116 ms | `transmitter/serial_log.csv` |
| Duty cycle used | 0.94% of a 1% allowance | `transmitter/serial_log.csv` |
| RSSI / SNR | −36 to −40 dBm / 9.75 to 10.75 dB | `receiver/receiver_serial_log.csv` |
| Inter-packet interval, TX board minus RX board | −1 to +4 ms (mean 1.5 ms; about 139 ppm relative drift) | both serial logs |
| FIFO queue length (first → last transmission) | 6 → 245 candidates | `transmitter/scheduler_decisions.csv` |
| Age of transmitted constraint at send (first / median / last) | 10 / 249 / 487 s | `transmitter/scheduler_decisions.csv` |

Paths are relative to `hardware/attached_hardware_evidence/session_hws_bacs_001/`. These results validate the LoRa airtime model and show that, under FIFO and a 1% duty cycle, the age of a transmitted constraint is set by queueing (minutes) rather than by the radio path (about 0.12 s). They do not evaluate range, mobility, map accuracy or schedulers other than FIFO.

**Notes on the session records.** The transmitter's clock-synchronization probe did not run on its Python 3.6 runtime, so the agreement of the two board clocks is used to confirm that both logs describe the same transmissions. The transmitter `manifest.json` records identical start and finish times; `session_info.txt` and the serial log give the session times (17:45:14Z start; packets 17:45:25–17:55:08Z). The session files are published unmodified so that the recorded checksums remain valid; they contain local host names and paths.

## 5. Photographs

Photographs are in `hardware/hardware_media/` and `demo/five_robots/images/`.

| File | Caption |
|---|---|
| `Robots_1.jpg`, `Robots3.jpg`, `Robots4.jpg`, `Robots5.jpg` | Robot platforms of the experimental setup on the laboratory bench, including the AgileX LIMO robot. |
| `Robots2.jpg`, `Robots7.jpg`, `Robots9.jpg`, `Robots10.jpg` | Robot platforms of the experimental setup, showing on-board computers, sensors and wiring. |
| `trial02_link_lora.jpg` | Screenshot of the transmitter and receiver terminals during a LoRa link trial. |
| `demo/five_robots/images/lora_modules/module_front.jpeg`, `module_detail.jpeg` | MKR WAN 1310 LoRa board used as a link endpoint. |
| `demo/five_robots/images/experimental_setup/hardware_inventory.jpeg`, `hardware_testing.jpeg` | LoRa boards before integration and bench testing with a multimeter. |

Photographs document the hardware; they do not establish timing, delivery or accuracy results.

## 6. Experimental videos

The original recordings are preserved by the authors. Public copies in `hardware/hardware_media/` have the audio removed (`*_no_audio.mp4`). Terminal recordings show local user and host names.

| Video | File | Duration | Caption |
|---|---|---|---|
| V1 | `HWS-BACS-001_20261120_194516_Live_Session_no_audio.mp4` | 32 s | Screen recording of the transmitter (FIFO, 11.0 s period) and receiver terminals at the start of session HWS-BACS-001; the received sequence numbers and signal strengths shown match the receiver log. |
| V2 | `Supplementary Video S2 — Live LoRa Transmission and Reception of BACS Constraints_no_audio.mp4`; `trial02_link_lora_video_no_audio.mp4` | 26 s; 308 s | Transmitter and receiver terminals during link trials earlier on the same day with the same boards (RSSI −42 to −46 dBm). Not part of the analysed session. |
| V3 | `lora_modules_alone_no_audio.mp4` | 6 s | The two MKR WAN 1310 boards with antennas used as link endpoints. |
| V4 | `Single_Robots_no_audio.mp4` | 32 s | Close-up of the AgileX LIMO robot platform. |
| V5 | `Single_Robots_movement_no_audio.mp4` | 12 s | The AgileX LIMO robot driving in the laboratory. The LoRa link session itself used stationary endpoints. |
| V6 | `Simulation demo_5_robors_map_no_audio.mp4` | 110 s | Five-robot ROS 2 Gazebo demonstration in a five-room office world with the occupancy map in RViz (`demo/five_robots/`). Illustrative; the reported simulation results come from `bacs_sim`. |
| V7 | `Two_Robots_Movement_no_audio.mp4` | 33 s | Two robot platforms, including the AgileX LIMO, driving in the laboratory. Illustrative; not part of the analysed link session. |

No video is used to compute any reported result, and no video establishes quantitative accuracy.

## 7. Available logs and supporting materials

| Path | Content |
|---|---|
| `attached_hardware_evidence/session_hws_bacs_001/transmitter/` | `serial_log.csv`, `scheduler_decisions.csv`, `session_info.txt`, `manifest.json`, `chronyc_tracking*.txt`, `clock_offsets.txt`, `SHA256SUMS.txt` |
| `attached_hardware_evidence/session_hws_bacs_001/receiver/` | `receiver_serial_log.csv`, `session_info.txt`, `manifest.json`, `chronyc_tracking.txt` |
| `firmware/` | Board firmware used for the link experiments |
| `HARDWARE_EVIDENCE_AUDIT.md`, `LAB_LOG.md` | Provenance audit and laboratory log |
| `../docs/HISTORICAL_DATASETS.md` | Status of historical hardware datasets |

## 8. Historical hardware datasets

The current public reproducibility package provides the simulation datasets, analysis scripts and available hardware experimental materials. Earlier development sessions (HWS-002, HWS-003 and HWS-005, July 2026) are historical hardware datasets that are not used for any reported result; their status and the findings of the provenance audit are described in [`docs/HISTORICAL_DATASETS.md`](../docs/HISTORICAL_DATASETS.md). They have not been independently reproduced.

## 9. Next physical experiments

Planned sessions: three or more moving robots, interleaved policies (FIFO, LIFO, random, BACS, BACS+), repeated sessions, a binding duty cycle, fused-pose logging and independent ground truth, captured with `scripts/record_session.sh` under unique session IDs with checksums.
