#include <Arduino.h>
#include <SoftwareSerial.h>

namespace {
constexpr uint8_t LINE_UART_RX_PIN = 10;
constexpr uint8_t LINE_UART_TX_PIN = 9;
constexpr long LINE_UART_BAUD = 19200;
constexpr uint8_t SENSOR_COUNT = 6;
constexpr uint8_t RAW_PACKET_SIZE = 8;
constexpr uint8_t FRAME_SYNC_0 = 0xA5;
constexpr uint8_t FRAME_SYNC_1 = 0x5A;

constexpr char CMD_READ_ONCE = 'R';
constexpr char CMD_STREAM_ON = 'S';
constexpr char CMD_STREAM_OFF = 'P';

constexpr unsigned long REQUEST_PERIOD_MS = 25;
constexpr unsigned long RX_TIMEOUT_US = 12000;
constexpr unsigned long SUMMARY_PERIOD_MS = 1000;
constexpr unsigned long STALE_WARN_MS = 250;

SoftwareSerial lineSerial(LINE_UART_RX_PIN, LINE_UART_TX_PIN);

uint32_t requestCount = 0;
uint32_t successCount = 0;
uint32_t timeoutCount = 0;
uint32_t badByteCount = 0;

unsigned long lastRequestMs = 0;
unsigned long lastSummaryMs = 0;
unsigned long lastGoodRxMs = 0;
uint16_t lastRaw[SENSOR_COUNT] = {0};

void unpackRawPacket(const uint8_t packet[RAW_PACKET_SIZE], uint16_t rawOut[SENSOR_COUNT]) {
  uint64_t packed = 0;
  for (uint8_t i = 0; i < RAW_PACKET_SIZE; i++) {
    packed |= (static_cast<uint64_t>(packet[i]) << (8u * i));
  }
  for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
    rawOut[i] = static_cast<uint16_t>((packed >> (10u * i)) & 0x03FFu);
  }
}

uint8_t checksumRaw(const uint8_t packet[RAW_PACKET_SIZE]) {
  uint8_t crc = 0;
  for (uint8_t i = 0; i < RAW_PACKET_SIZE; i++) {
    crc ^= packet[i];
  }
  return crc;
}

bool requestRaw(uint16_t rawOut[SENSOR_COUNT]) {
  while (lineSerial.available() > 0) {
    lineSerial.read();
  }

  lineSerial.write(static_cast<uint8_t>(CMD_READ_ONCE));
  requestCount++;

  uint8_t packet[RAW_PACKET_SIZE];
  uint8_t got = 0;
  bool gotSync0 = false;
  bool gotSync1 = false;
  const unsigned long start = micros();
  while (micros() - start < RX_TIMEOUT_US) {
    while (lineSerial.available() > 0) {
      const int rx = lineSerial.read();
      if (rx < 0) continue;
      const uint8_t b = static_cast<uint8_t>(rx);

      if (!gotSync0) {
        gotSync0 = (b == FRAME_SYNC_0);
        continue;
      }
      if (!gotSync1) {
        gotSync1 = (b == FRAME_SYNC_1);
        if (!gotSync1) {
          gotSync0 = (b == FRAME_SYNC_0);
        }
        continue;
      }

      if (got < RAW_PACKET_SIZE) {
        packet[got++] = b;
        continue;
      }

      const uint8_t expectedCrc = checksumRaw(packet);
      if (b != expectedCrc) {
        badByteCount++;
        gotSync0 = false;
        gotSync1 = false;
        got = 0;
        continue;
      }

      unpackRawPacket(packet, rawOut);
      return true;
    }
  }

  timeoutCount++;
  return false;
}

void printRaw(const uint16_t raw[SENSOR_COUNT]) {
  Serial.print(F("raw A0..A5="));
  for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
    Serial.print(raw[i]);
    if (i < SENSOR_COUNT - 1) Serial.print(',');
  }
  Serial.println();
}

void printSummary() {
  const uint32_t total = requestCount;
  const float okPct = (total > 0) ? (100.0f * static_cast<float>(successCount) / static_cast<float>(total)) : 0.0f;

  Serial.print(F("[summary] req="));
  Serial.print(requestCount);
  Serial.print(F(" ok="));
  Serial.print(successCount);
  Serial.print(F(" timeout="));
  Serial.print(timeoutCount);
  Serial.print(F(" bad="));
  Serial.print(badByteCount);
  Serial.print(F(" ok%="));
  Serial.println(okPct, 1);
}

} // namespace

void setup() {
  Serial.begin(115200);
  lineSerial.begin(LINE_UART_BAUD);

  // Force request/response mode for deterministic tests.
  lineSerial.write(static_cast<uint8_t>(CMD_STREAM_OFF));
  delay(20);
  while (lineSerial.available() > 0) {
    lineSerial.read();
  }

  Serial.println(F("line_link_test ready"));
  Serial.print(F("pins: RX="));
  Serial.print(LINE_UART_RX_PIN);
  Serial.print(F(" TX="));
  Serial.print(LINE_UART_TX_PIN);
  Serial.print(F(", baud="));
  Serial.println(LINE_UART_BAUD);
  Serial.println(F("expected: line_reader firmware with packed raw R/S/P protocol"));
}

void loop() {
  const unsigned long now = millis();

  if (now - lastRequestMs >= REQUEST_PERIOD_MS) {
    lastRequestMs = now;

    uint16_t raw[SENSOR_COUNT] = {0};
    if (requestRaw(raw)) {
      successCount++;
      lastGoodRxMs = now;
      for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
        lastRaw[i] = raw[i];
      }
      printRaw(raw);
    }
  }

  if (lastGoodRxMs > 0 && (now - lastGoodRxMs > STALE_WARN_MS)) {
    Serial.print(F("[warn] no response for "));
    Serial.print(now - lastGoodRxMs);
    Serial.println(F(" ms"));
    lastGoodRxMs = now;
  }

  if (now - lastSummaryMs >= SUMMARY_PERIOD_MS) {
    lastSummaryMs = now;
    printSummary();
    Serial.print(F("[last] "));
    printRaw(lastRaw);
  }
}
