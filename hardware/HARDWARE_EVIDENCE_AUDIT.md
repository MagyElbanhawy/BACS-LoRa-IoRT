# Hardware evidence audit

Audit date: 2026-11-22

This audit documents the provenance and verification of the physical hardware records, keeping hardware evidence distinct from the simulation results.

## Evidence reviewed

### FIFO transmitter package

The supplied ZIP has SHA-256 `80e5e28b812da94f11a408e5d356c722400fb12571610651e85b9a1be5e39224`. All seven hashes recorded inside `SHA256SUMS.txt` match their members.

The separately supplied unpacked copies also pass all seven recorded checksums and are byte-for-byte identical to the corresponding ZIP members. They are duplicates, not an additional run.

The package contains a transmitter manifest, session information, clock records, 54 scheduler decisions, and a serial log. The evidence supports these narrow facts:

- radio configuration: 868 MHz, SF7, 125 kHz, coding rate 4/5, 52-byte payload;
- 300 constraints generated;
- 54 FIFO selections and 54 `HOST_TX`/`MR_TX_DONE` pairs;
- device-reported transmit airtime of 103–104 ms.

Limitations:

- no receiver log or rosbag is inside the ZIP;
- the transmitter's clock synchronization probe did not run because its Python 3.6 runtime did not support the invoked `capture_output` argument;
- `manifest.json` records identical start and finish times at 17:55:14Z, whereas `session_info.txt` says 17:45:14Z and the serial data span approximately 17:45:25–17:55:08Z;
- the session information exposes a hostname and therefore must be sanitized in any anonymous copy.

Conclusion: this package independently supports the **54-transmission FIFO transmitter count** and 103–104 ms device-reported airtime. It does not, by itself, prove delivery at the receiver or any SLAM result.

### Combined `HWS-BACS-001` receiver directory

The receiver directory combines records from more than one acquisition:

- the receiver manifest starts on 2026-11-20, but `serial_log.csv` and the populated rosbag start on 2026-11-18;
- the populated rosbag is internally readable, but contains 18,462 messages while its metadata declares 18,463; `/bacs/candidates` contains 299 messages rather than the declared 300;
- a second receiver rosbag is structurally valid but contains zero messages;
- `receiver_serial_log.csv` contains 73 valid RX rows, covering 54 unique sequence numbers, plus one later noise row;
- the receiver log includes 11 pre-run rows before the sequence beginning near 17:45, and the likely FIFO segment has 54 unique receptions plus eight immediate duplicate rows;
- all 73 valid receiver payloads match the deterministic FIFO transmitter payload for their sequence number, showing that multiple attempts/runs using the same payload stream were appended;
- there is no directory-level checksum manifest;
- the session information exposes a hostname, local user path, and repository URL.

The separate 2026-11-18 transmitter log contains 54 `HOST_TX`, 54 `MR_TX_DONE`, and 54 delivery-ACK rows, with mean device-reported airtime 108.600 ms. Its provenance is incomplete because it is stored under a receiver manifest dated two days later.

Conclusion (2026-11-22): preserve these files unchanged and use them only together with a documented segmentation and clock-agreement procedure. This procedure was completed on 2026-11-24 (see *Resolution* below).

### Aggregate RAR

The RAR is readable and has SHA-256 `e9397558dc70cefb526041d4199eca191d2954ba76e6cb5c75f5a474631348df`. It contains many short `HWS-NEW-*` runs, link trials, and an appended receiver dataset.

These exploratory logs have varying formats and appended retry attempts. They are superseded by session HWS-BACS-001 and are not included in the current public release.

### Images and videos

Nine JPEGs and six MP4s were found in `hardware/hardware_media/`.

The data owner states that the videos record the same receiver acquisition represented by `session_info.txt`, `chronyc_tracking.txt`, `manifest.json`, and `receiver_serial_log.csv`. This is retained as a provenance statement, not treated as an independent run or independent measurement. The videos were not synchronized to individual CSV rows or used to repair/segment the mixed receiver log.

- The JPEG decoders expose no EXIF tags, but several photographs contain an institutional document/logo in the scene.
- The LoRa screenshot visibly exposes local user paths and session details.
- The photographs show robot hardware only; they do not establish experiment policy, timestamps, delivered counts, ground truth, fused trajectories, or RMSE.
- Muted, audio-free copies of all experimental videos (`*_no_audio.mp4`) have been generated for anonymous academic review and online streaming compatibility.

## Resolution (2026-11-24)

The manuscript uses the 2026-11-20 transmitter package together with the segment of `receiver_serial_log.csv` that starts near 17:45Z:

- The segment contains 54 unique sequence numbers, each with a payload byte-identical to the transmitted one (54 of 54 received).
- Excluded rows are listed in the segmentation table of the Supplementary Material: 8 immediate double reports (20–21 ms after the first report), 11 rows from test transmissions 100–305 s before the session, and 1 noise frame (−118 dBm) about 33 minutes after it.
- The inter-packet intervals measured by the two boards agree to −1 to +4 ms per packet (mean 1.5 ms, about 139 ppm relative drift), which confirms that the transmitter and receiver records describe the same transmissions without requiring host-clock alignment.
- The rosbags and the 2026-11-18 transmitter log are not used. No end-to-end latency, packet-age decomposition or map result is derived from this session.

The public folder `session_hws_bacs_001/` contains the transmitter package and the receiver serial log, session information, manifest and clock record, unmodified.

## Claim status

| Claim | Status after this audit |
|---|---|
| 54 FIFO transmissions | Supported at the transmitter by the checksummed ZIP |
| 103–104 ms airtime for the supplied FIFO package | Supported as device-reported airtime |
| 55 delivered and 10 duplicates in a separate link test | Not used; independent reproduction pending |
| 54 of 54 packets received in HWS-BACS-001 | Supported after segmentation (see *Resolution*) |
| Physical packet-age decomposition | Requires a paired TX/RX session with verified clock alignment |
| Physical map/pose RMSE | Requires additional experimental verification; verified ground truth and fused trajectories are not included in the current archive |
| Real-robot superiority of BACS/BACS+ | Requires additional experimental verification |

## Practices for future sessions

1. Copy originals into immutable session folders without editing their bytes and generate a top-level checksum manifest.
2. Give each acquisition attempt a unique public session ID; never append retries under one ID.
3. Pair transmitter and receiver logs using a shared run UUID in every row and record the actual host-clock offset before and after each run.
4. Regenerate rosbag metadata from the bag and verify topic counts against SQLite.
5. Keep the original combined receiver log, but publish a separate derived segmentation table that identifies excluded pre-run, duplicate, and corrupt rows.
6. Sanitize hostnames, local paths, repository identity, logos, and visible screens only in reviewer copies. Keep originals private and unchanged.
7. Leave videos out of the anonymous package unless a muted, visually anonymized derivative is created and reviewed.
