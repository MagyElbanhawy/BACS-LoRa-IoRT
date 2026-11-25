/*
 * MR_debug.ino - diagnostic build of the MR firmware.
 *
 * Identical wire protocol to MR_portable.ino, but with verbose Serial
 * prints before and after each LoRa call. Use this ONLY for debugging the
 * "I type T,1,AA and the board resets / disconnects" symptom; switch back
 * to MR_portable.ino for real captures so the serial_log.csv format is
 * not polluted with diagnostic lines.
 *
 * Symptoms this build helps you localize:
 *   - "READY" prints, then typing T,1,AA causes the board to reset
 *     (you see "READY" again after typing) -> crash in the TX path
 *   - "ERROR: LoRa radio init failed" -> wrong SPI pins or wiring
 *   - Hangs at "TX: calling beginPacket" -> LoRa.write() is blocking
 */

#include <LoRa.h>

const float    RF_FREQ_HZ       = 868.0e6;
const uint8_t  TX_POWER_DBM     = 14;
const uint8_t  SPREADING_FACTOR = 7;
const uint32_t SIGNAL_BW_HZ     = 125000;
const uint8_t  CODING_RATE_4    = 5;
const uint8_t  PREAMBLE_LEN     = 8;
const uint8_t  PAYLOAD_LEN      = 52;

uint8_t buf[PAYLOAD_LEN];

void setup() {
  Serial.begin(115200);
  while (!Serial) { ; }
  Serial.println("BOOT");

  Serial.print("LoRa.begin: ");
  if (!LoRa.begin(RF_FREQ_HZ)) {
    Serial.println("FAIL");
    Serial.println("ERROR: LoRa radio init failed. Check wiring / SPI pins.");
    while (true) { ; }
  }
  Serial.println("OK");

  LoRa.setTxPower(TX_POWER_DBM);
  LoRa.setSpreadingFactor(SPREADING_FACTOR);
  LoRa.setSignalBandwidth(SIGNAL_BW_HZ);
  LoRa.setCodingRate4(CODING_RATE_4);
  LoRa.setPreambleLength(PREAMBLE_LEN);
  LoRa.receive();

  Serial.println("READY");
}

void loop() {
  if (Serial.available() > 0) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    Serial.print("DBG recv: '");
    Serial.print(cmd);
    Serial.print("' (len=");
    Serial.print(cmd.length());
    Serial.println(")");

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

        Serial.print("DBG seq=");
        Serial.print(seqFull);
        Serial.print(" (payload byte=");
        Serial.print(seq);
        Serial.print(") hexLen=");
        Serial.print(hexStr.length());
        Serial.print(" buf[0..3]=");
        for (int i = 0; i < 4; i++) {
          if (buf[i] < 0x10) Serial.print('0');
          Serial.print(buf[i], HEX);
          Serial.print(' ');
        }
        Serial.println();

        Serial.println("DBG calling LoRa.beginPacket()");
        LoRa.beginPacket();
        Serial.println("DBG calling LoRa.write(52)");
        size_t n = LoRa.write(buf, PAYLOAD_LEN);
        Serial.print("DBG write returned ");
        Serial.println(n);
        Serial.println("DBG calling LoRa.endPacket()");
        uint32_t t0 = millis();
        LoRa.endPacket();
        Serial.print("DBG endPacket done at ms=");
        Serial.println(t0);

        Serial.print("TX,");
        Serial.print(t0);
        Serial.print(",");
        Serial.print(seqFull);
        Serial.println(",SENT");

        LoRa.receive();
        Serial.println("DBG back in RX mode");
      } else {
        Serial.println("DBG: missing second comma");
      }
    } else {
      Serial.println("DBG: not a T, command");
    }
  }
}

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
