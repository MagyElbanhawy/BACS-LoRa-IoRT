/*
 * MR_portable.ino - portable MR firmware for the BACS-ROS2-SLAM testbed.
 *
 * Wire protocol (identical to hardware/firmware/MR/MR.ino):
 *
 *     Host -> MR :  T,<seq>,<hex_payload>\n
 *     MR   -> Host:
 *         TX,<timestamp_ms>,<seq>,STATUS\n          // TX acknowledgement
 *         RX,<timestamp_ms>,<seq>,<rssi>,<snr>\n    // inbound packet
 *
 * This portable build uses the sandeepmistry/LoRa library (Arduino Library
 * Manager -> search "LoRa" by Sandeep Mistry). It works with any board that
 * can drive an SX1276/RFM95-class radio over SPI:
 *
 *   - Arduino UNO/Nano/Mega + RFM95 wing
 *   - Arduino MKR boards + external SX1276 module
 *   - ESP32 (Heltec WiFi LoRa 32, TTGO LoRa32, etc.)
 *
 * INSTALL:
 *   1. Arduino IDE -> Tools -> Manage Libraries... -> search "LoRa"
 *      by Sandeep Mistry -> Install.
 *   2. Tools -> Board -> <your board>.
 *   3. Tools -> Port -> <your board's serial port>.
 *   4. Sketch -> Upload.
 *
 * DEFAULT SPI PINS (override with LoRa.setPins(ss, rst, dio0) in setup()
 * if your wiring differs):
 *
 *   Board            SS    RST   DIO0   SCK   MISO  MOSI
 *   ---------------------------------------------------------
 *   Arduino UNO/Nano  10    9     2      13    12     11
 *   Arduino Mega      53    48    2      52    50     51
 *   Arduino MKR       5     6     2      9     8      7
 *   ESP32 (Heltec)    18    14    26     5     19     27
 *   ESP32 (TTGO)      18    14    26     5     19     27
 *
 * The sandeepmistry LoRa library auto-selects SPI pins for known boards;
 * only call LoRa.setPins() if your module is wired to non-default pins.
 */

#include <LoRa.h>

// ---------------------------------------------------------------------------
// Radio parameters - MUST match config/lora.yaml so the host-side airtime
// calculation (lora_airtime_s in scheduler.py) matches the firmware's actual
// on-air time. Change config/lora.yaml if you change any of these.
// ---------------------------------------------------------------------------
const float    RF_FREQ_HZ       = 868.0e6;  // EU 868 MHz ISM band
const uint8_t  TX_POWER_DBM     = 14;
const uint8_t  SPREADING_FACTOR = 7;
const uint32_t SIGNAL_BW_HZ     = 125000;
const uint8_t  CODING_RATE_4    = 5;         // 4/5
const uint8_t  PREAMBLE_LEN     = 8;
const uint8_t  PAYLOAD_LEN      = 52;        // 1 seq byte + 51 payload bytes

uint8_t buf[PAYLOAD_LEN];

void setup() {
  Serial.begin(115200);
  // Block until the host serial port attaches (USB CDC on MKR/ESP32).
  while (!Serial) { ; }

  // Optional: override SPI pins for non-default wiring.
  // LoRa.setPins(ss, reset, dio0);

  if (!LoRa.begin(RF_FREQ_HZ)) {
    Serial.println("ERROR: LoRa radio init failed. Check wiring.");
    while (true) { ; }   // halt so the host sees no READY banner
  }

  LoRa.setTxPower(TX_POWER_DBM);
  LoRa.setSpreadingFactor(SPREADING_FACTOR);
  LoRa.setSignalBandwidth(SIGNAL_BW_HZ);
  LoRa.setCodingRate4(CODING_RATE_4);
  LoRa.setPreambleLength(PREAMBLE_LEN);
  LoRa.receive();   // enter continuous receive mode

  // READY banner - the ROS 2 node waits for this before scheduling.
  Serial.println("READY");
}

void loop() {
  // ----------------------------------------------------------------- RX path
  // A LoRa packet arrived; emit RX,timestamp_ms,seq,rssi,snr so the host can
  // log it in serial_log.csv alongside its own wall-clock capture.
  int packetSize = LoRa.parsePacket();
  if (packetSize > 0) {
    int n = 0;
    while (LoRa.available() && n < PAYLOAD_LEN) {
      buf[n++] = (uint8_t)LoRa.read();
    }
    // Zero-pad if short (firmware always logs PAYLOAD_LEN bytes).
    while (n < PAYLOAD_LEN) { buf[n++] = 0; }

    // Extract the full seq (first 4 bytes of payload, big-endian uint32).
    // This matches the constraint payload layout in constraint_source.py.
    uint32_t rxSeq = ((uint32_t)buf[0] << 24) | ((uint32_t)buf[1] << 16) |
                     ((uint32_t)buf[2] << 8) | (uint32_t)buf[3];

    Serial.print("RX,");
    Serial.print(millis());
    Serial.print(",");
    Serial.print(rxSeq);                   // full seq (uint32, no wraparound)
    Serial.print(",");
    Serial.print(LoRa.packetRssi());
    Serial.print(",");
    Serial.print(LoRa.packetSnr());
    Serial.print(",");
    // Append the 52-byte payload as hex so the host can decode the
    // full constraint data (robot_i, robot_j, dx, dy, trust, info, etc.).
    for (int i = 0; i < PAYLOAD_LEN; i++) {
      if (buf[i] < 0x10) Serial.print('0');
      Serial.print(buf[i], HEX);
    }
    Serial.println();
  }

  // ----------------------------------------------------------------- TX path
  // Host writes:  T,<seq>,<hex_payload>\n
  // MR echoes a TX acknowledgement the instant endPacket() returns so the
  // host can compute a per-packet clock offset.
  if (Serial.available() > 0) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd.startsWith("T,")) {
      int firstComma  = cmd.indexOf(',');
      int secondComma = cmd.indexOf(',', firstComma + 1);
      if (secondComma > 0) {
        String seqStr = cmd.substring(firstComma + 1, secondComma);
        String hexStr  = cmd.substring(secondComma + 1);

        long seqFull = seqStr.toInt();   // full seq value (no 8-bit wrap)

        // The constraint payload starts with a 4-byte big-endian uint32 seq.
        // Write all 4 bytes so the receiver's rxSeq extraction matches.
        // The hex payload from the host also starts with the seq (4 bytes),
        // so we write the seq here and parse the hex starting at byte 4.
        buf[0] = (uint8_t)((seqFull >> 24) & 0xFF);
        buf[1] = (uint8_t)((seqFull >> 16) & 0xFF);
        buf[2] = (uint8_t)((seqFull >> 8)  & 0xFF);
        buf[3] = (uint8_t)( seqFull        & 0xFF);
        // Parse the hex payload into buf[4..51] (48 bytes = 96 hex chars).
        // The first 8 hex chars (4 bytes) of hexStr are the seq, which we
        // already wrote above, so skip them.
        String hexAfterSeq = hexStr.substring(8);   // skip first 4 bytes (seq)
        parse_hex_into(hexAfterSeq, buf + 4, PAYLOAD_LEN - 4);

        uint32_t t0 = millis();
        LoRa.beginPacket();
        LoRa.write(buf, PAYLOAD_LEN);
        LoRa.endPacket();

        // TX,timestamp_ms,seq,STATUS - the host logs this in serial_log.csv
        // and uses (mr_timestamp_ms, host_handoff_ns) for clock_offsets.txt.
        // Print the full (unwrapped) seq so the host's transport.py
        // parsed.seq == seq match succeeds past packet 255.
        Serial.print("TX,");
        Serial.print(t0);
        Serial.print(",");
        Serial.print(seqFull);
        Serial.println(",SENT");

        // Re-enter receive mode (endPacket() leaves the radio idle).
        LoRa.receive();
      }
    }
  }
}

// Parse the host's hex payload into the radio TX buffer. Each pair of hex
// digits produces one byte. Short payloads are zero-padded to (max_bytes).
static void parse_hex_into(const String &hex, uint8_t *out, size_t max_bytes) {
  size_t written = 0;
  for (size_t i = 0; i + 1 < hex.length() && written < max_bytes; i += 2) {
    char hi = hex.charAt(i);
    char lo = hex.charAt(i + 1);
    out[written++] = (uint8_t)((hex_nibble(hi) << 4) | hex_nibble(lo));
  }
  while (written < max_bytes) {
    out[written++] = 0;
  }
}

static uint8_t hex_nibble(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  return 0;
}
