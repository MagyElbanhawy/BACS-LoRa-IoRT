# Hardware laboratory log

## Session HWS-BACS-001 (2026-11-20)

| Item | Record |
|---|---|
| Purpose | Link-level measurement of LoRa airtime, delivery, board-clock agreement and FIFO backlog growth |
| Set-up | Two stationary endpoints (labelled LIMO-01 transmitter and LIMO-02 receiver), Arduino MKR WAN 1310 boards, 868 MHz, SF7, 125 kHz, CR 4/5, 52-byte payload |
| Policy | FIFO; synthetic candidate stream, one candidate every 2 s; one packet every 11.0 s; 600 s |
| Transmitter records | Checksummed package: 300 candidates, 54 decisions and 54 transmissions, board-reported airtime 103–104 ms |
| Receiver records | `receiver_serial_log.csv`; the session segment has 54 unique sequence numbers |
| Deviations | Receiver log also holds 11 rows from earlier test transmissions, 8 double reports and 1 later noise frame (excluded; listed in the Supplementary segmentation table). The transmitter clock-synchronization probe did not run (Python 3.6). The transmitter manifest records identical start and finish times. |
| Not used | Receiver rosbags; a 2026-11-18 transmitter log stored in the receiver directory |
| Video | V1 (`hardware_media/HWS-BACS-001_20261120_194516_Live_Session_no_audio.mp4`) shows the start of the session; it is contextual documentation and not used for any measurement |

Details and integrity checks: [`HARDWARE_EVIDENCE_AUDIT.md`](HARDWARE_EVIDENCE_AUDIT.md).

## Historical sessions

HWS-002, HWS-003, HWS-005 (July 2026) and the HWS-1xx test fixtures are historical testbed development records and are not part of the current public release; see [`../docs/HISTORICAL_DATASETS.md`](../docs/HISTORICAL_DATASETS.md).

## Release rule

Original raw files are never edited. A future session must have a unique ID, unmodified TX/RX logs, a consistent bag with regenerated metadata, a manifest, clock evidence, deviation notes and SHA-256 checksums.
