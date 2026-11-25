# MKR WAN 1310 Link-Test Firmware

`MR_portable.ino` is a symmetric point-to-point test image: flash the same
sketch to the sender and receiver boards.

## Hardware requirement

Attach a compatible 868 MHz antenna to the micro-U.FL connector on **each**
MKR WAN 1310 before powering or transmitting. Ensure external antennas are securely connected to the micro-U.FL ports prior to transmission tests to prevent RF impedance mismatch and packet degradation.

In Arduino IDE select **Arduino MKR WAN 1310**. Do not add the generic external
radio mapping `LoRa.setPins(5, 6, 2)`; the LoRa library provides board-specific
SPI1 and internal radio definitions.

## Evidence protocol

```text
Host -> board: T,<sequence>,<48-byte-hex-payload>

Board -> host:
TX_DONE,<device_uptime_ms>,<sequence>,<tx_duration_us>
ACK_RX,<device_uptime_ms>,<sequence>,<rssi_dbm>,<snr_db>
ACK_TIMEOUT,<device_uptime_ms>,<sequence>
RX,<device_uptime_ms>,<sequence>,<rssi_dbm>,<snr_db>
ACK_TX,<device_uptime_ms>,<sequence>,<tx_duration_us>
```

`TX_DONE` proves only that the local radio finished its transmit operation.
It is not delivery evidence. `ACK_RX` is the success condition because the
remote board sends it only after emitting an `RX` record for the matching
52-byte data frame.

Device uptime is deliberately kept separate from host time. `timing_log.csv`
uses the host monotonic clock for intervals and records the board-reported
transmit duration without inventing a cross-clock offset.

## Radio configuration

| Parameter | Value |
|---|---:|
| Frequency | 868 MHz |
| Power | 14 dBm |
| Spreading factor | 7 |
| Bandwidth | 125 kHz |
| Coding rate | 4/5 |
| Preamble | 8 symbols |
| Sync word | 0x12 |
| CRC | enabled |
| Data frame | 52 bytes |

At these settings the configured data airtime is 102.656 ms. A 1% duty-cycle
link test therefore uses an interval of at least 10.266 seconds; the runner
defaults to 11 seconds.

Run `tools/receiver_logger.py` on the Raspberry Pi to save the receiver's
`RX`/RSSI/SNR and `ACK_TX` evidence.
