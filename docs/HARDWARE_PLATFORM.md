# Hardware platform

The physical component is a two-robot ROS 2 Humble platform with LoRa inter-robot communication. Full documentation, photographs, videos and logs are in [`hardware/README.md`](../hardware/README.md).

| Item | Used in session HWS-BACS-001 |
|---|---|
| Robot platforms | AgileX LIMO platform; endpoints labelled LIMO-01 and LIMO-02 in the logs; stationary during the session |
| Radios | Two Arduino MKR WAN 1310 boards (Murata CMWX1ZZABZ, Semtech SX1276), firmware `hardware/firmware/MR/MR.ino` |
| LoRa settings | 868 MHz, SF7, 125 kHz, CR 4/5, 14 dBm, 52-byte payload (102.7 ms model time on air) |
| Duty cycle | Enforced in software: one packet every 11.0 s (0.94%) |
| Scheduler | FIFO, synthetic candidate stream (one candidate every 2 s), 600 s |
| Ground truth | None; the session measures link timing and delivery, not map accuracy |

The ROS 2 code in `ros2_ws/src/bacs_scheduler/` also contains a REYAX RYLR998 interface (`rylr998.py`, `config/lora.yaml`) for the planned policy sessions described in [`HARDWARE_EXPERIMENT_V2.md`](HARDWARE_EXPERIMENT_V2.md).
