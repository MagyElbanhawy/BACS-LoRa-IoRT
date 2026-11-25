/*
 * MR.ino - MR (Mobile-Robot radio) firmware for the BACS-ROS2-SLAM testbed.
 *
 * Replaces the historical REYAX RYLR998 AT-command dialect
 *     AT+SEND=<addr>,<len>,<hex>
 * with a minimal, transparent, host-driven binary-line protocol:
 *
 *     Host -> MR :  T,<seq>,<hex_payload>\n
 *     MR   -> Host:
 *         TX,<timestamp_ms>,<seq>,<STATUS>\n     // TX acknowledgement
 *         RX,<timestamp_ms>,<seq>,<rssi>,<snr>\n  // inbound packet
 *
 * The MR (was: "MKR" in earlier revisions of this sketch) label refers to the
 * mobile-robot-side radio mote. Rename changes:
 *     MKR_Radio.h   -> MR_Radio.h   (drop-in alias provided below)
 *     LoRaRadio radio -> LoRaRadio radio  (class unchanged; only the human
 *                                          label "MKR" was renamed to "MR")
 *
 * Hardware target: Arduino MKR WAN 1310 (or any board with a compatible
 * SX1276-class radio). Wire the radio's SPI to the board's default SPI pins.
 */

#include <SPI.h>

// The historical sketch included <MKR_Radio.h>. The radio driver class is
// identical; only the human-facing label has been renamed from "MKR" to "MR".
// If your toolchain still ships MKR_Radio.h, the alias below keeps the sketch
// buildable without driver changes.
#if __has_include(<MR_Radio.h>)
  #include <MR_Radio.h>
#elif __has_include(<MKR_Radio.h>)
  #include <MKR_Radio.h>
  // Provide a backwards-compatible alias so the rest of the sketch can refer
  // to MR_Radio without caring which header was actually included.
  using MR_Radio = LoRaRadio;
#else
  #error "Neither MR_Radio.h nor MKR_Radio.h found. Install the MR radio driver."
#endif

LoRaRadio radio;

const uint8_t PAYLOAD_LEN = 52;
uint8_t buf[PAYLOAD_LEN];

// Default radio parameters - must match config/lora.yaml exactly so that the
// host-side airtime calculation (lora_airtime_s in scheduler.py) stays
// consistent with the firmware's actual on-air time.
const float    RF_FREQ_HZ       = 868.0e6;
const uint8_t  TX_POWER_DBM     = 14;
const uint8_t  SPREADING_FACTOR = 7;
const uint32_t SIGNAL_BW_HZ     = 125000;
const uint8_t  CODING_RATE_4    = 5;       // 4/5
const uint8_t  PREAMBLE_LEN     = 8;

void setup() {
  Serial.begin(115200);
  // Wait for the host serial port to attach. The ROS 2 sender will only
  // start issuing T,seq,hex lines once it sees the READY banner below.
  while (!Serial) { ; }

  radio.begin(RF_FREQ_HZ);
  radio.setTxPower(TX_POWER_DBM);
  radio.setSpreadingFactor(SPREADING_FACTOR);
  radio.setSignalBandwidth(SIGNAL_BW_HZ);
  radio.setCodingRate4(CODING_RATE_4);
  radio.setPreambleLength(PREAMBLE_LEN);
  radio.setPacketLength(PAYLOAD_LEN);
  radio.receive();

  // READY banner - the ROS 2 node waits for this line before scheduling.
  Serial.println("READY");
}

void loop() {
  // ----------------------------------------------------------------- RX path
  // A LoRa packet arrived; emit an RX,timestamp_ms,seq,rssi,snr line so the
  // host can log it in serial_log.csv alongside its own wall-clock capture.
  if (radio.parsePacket()) {
    int len = radio.read(buf, PAYLOAD_LEN);
    (void)len;  // we always read PAYLOAD_LEN; len is informational only

    Serial.print("RX,");
    Serial.print(millis());
    Serial.print(",");
    Serial.print(buf[0]);                 // seq
    Serial.print(",");
    Serial.print(radio.packetRSSI());
    Serial.print(",");
    Serial.println(radio.packetSNR());
  }

  // ----------------------------------------------------------------- TX path
  // The host writes:   T,<seq>,<hex_payload>\n
  // The MR echoes a TX acknowledgement the instant the radio's endPacket()
  // returns, so that the host can compute a per-packet clock offset.
  if (Serial.available() > 0) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd.startsWith("T,")) {
      int firstComma  = cmd.indexOf(',');
      int secondComma = cmd.indexOf(',', firstComma + 1);
      if (secondComma > 0) {
        String seqStr = cmd.substring(firstComma + 1, secondComma);
        String hexStr  = cmd.substring(secondComma + 1);

        uint8_t seq = (uint8_t)seqStr.toInt();   // for the payload byte (0-255)
        long   seqFull = seqStr.toInt();           // for the ack line (full value, no wrap)
        buf[0] = seq;
        parse_hex_into(hexStr, buf + 1, PAYLOAD_LEN - 1);

        uint32_t t0 = millis();
        radio.beginPacket();
        radio.write(buf, PAYLOAD_LEN);
        radio.endPacket();

        // TX,timestamp_ms,seq,STATUS - the host logs this in serial_log.csv
        // and uses (mr_timestamp_ms, host_handoff_ns) for clock_offsets.txt.
        // Print the full (unwrapped) seq so the host's transport.py
        // parsed.seq == seq match succeeds past packet 255.
        Serial.print("TX,");
        Serial.print(t0);
        Serial.print(",");
        Serial.print(seqFull);
        Serial.println(",SENT");
      }
    }
  }
}

// Parse the host's hex payload into the radio TX buffer. Each pair of hex
// digits produces one byte. Short payloads are zero-padded to PAYLOAD_LEN-1.
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
