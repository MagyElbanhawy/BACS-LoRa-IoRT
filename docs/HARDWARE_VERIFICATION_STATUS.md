# Status of hardware results

This document lists which hardware results are supported by the current release and which require additional experimental verification.

## Supported by the current release

Session HWS-BACS-001 (two stationary LoRa endpoints, FIFO, 600 s): 54 of 54 packets delivered, 103–104 ms board-reported airtime (model 102.7 ms), 0.94% duty cycle, −1 to +4 ms TX/RX interval agreement, FIFO backlog growth to 487 s constraint age. Evidence and file list: [`hardware/README.md`](../hardware/README.md).

## Requiring additional experimental verification

- Physical map-alignment accuracy and real-robot comparison of FIFO, BACS and BACS+: no fused-pose trajectories with verified ground truth are available; these claims are not made in the current manuscript.
- Results based on the historical sessions HWS-002, HWS-003 and HWS-005: see [`docs/HISTORICAL_DATASETS.md`](HISTORICAL_DATASETS.md) for the audit qualifications.

## Planned measured sessions

New sessions are captured with `scripts/record_session.sh` under `~/hardware_runs/<session_id>/` and produce `serial_log.csv`, `rosbag.db3` (with fused poses) and `clock_offsets.txt`. The planned design interleaves FIFO, BACS, BACS+ and random ordering so that environmental drift is shared across policies. Supporting components:

| Component | Path |
|---|---|
| Capture script | `scripts/record_session.sh` |
| `T,seq,hex` serial sender | `ros2_ws/src/bacs_scheduler/bacs_scheduler/transport.py` (`TSeqHexSerialSender`) |
| Board firmware with timestamped acknowledgements | `hardware/firmware/MR/MR.ino` |
| Extended scheduler log schema | `ros2_ws/src/bacs_scheduler/bacs_scheduler/scheduler_logging.py` |
| Random-ordering policy | `config/random.yaml` and the scheduler node |

## Notes on file formats and statistics

For historical sessions, `.db3` files are the original ROS 2 recordings and `.mcap` files are later conversions. Historical policy sessions were recorded on separate days, so any comparison between them treats runs as independent samples.
