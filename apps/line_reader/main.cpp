#include <Arduino.h>
#include <SoftwareSerial.h>

// Digital pins used for the inter-board UART link.
constexpr uint8_t SOFTSER_RX_PIN = 10;
constexpr uint8_t SOFTSER_TX_PIN = 9;
constexpr long SOFTSER_BAUD = 19200;

constexpr uint8_t IR_EMITTER_PIN = 12;

constexpr uint8_t SENSOR_COUNT = 6;
constexpr uint8_t SENSOR_PINS[SENSOR_COUNT] = {A0, A1, A2, A3, A4, A5};
constexpr uint8_t RAW_PACKET_SIZE = 8;
constexpr uint8_t FRAME_SYNC_0 = 0xA5;
constexpr uint8_t FRAME_SYNC_1 = 0x5A;
constexpr bool DEBUG_SERIAL = true;
constexpr unsigned long DEBUG_PERIOD_MS = 500;

// Keep the mask fresh at high rate.
constexpr unsigned long SAMPLE_INTERVAL_US = 1500;

// Protocol:
//   'R' -> reply once with packed raw packet (8 bytes for 6x10-bit values)
//   'S' -> start continuous raw packet streaming
//   'P' -> pause streaming
constexpr char CMD_READ_ONCE = 'R';
constexpr char CMD_STREAM_ON = 'S';
constexpr char CMD_STREAM_OFF = 'P';
constexpr unsigned long STREAM_INTERVAL_MS = 8;

SoftwareSerial softSerial(SOFTSER_RX_PIN, SOFTSER_TX_PIN);
uint16_t latestRaw[SENSOR_COUNT] = {0};
bool streamEnabled = false;
unsigned long lastSampleUs = 0;
unsigned long lastStreamMs = 0;
unsigned long lastDebugMs = 0;
uint32_t cmdReadOnceCount = 0;
uint32_t cmdStreamOnCount = 0;
uint32_t cmdStreamOffCount = 0;
uint32_t packetsSentCount = 0;
uint32_t ignoredCmdCount = 0;

void sampleRaw(uint16_t rawOut[SENSOR_COUNT]) {
  for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
    rawOut[i] = static_cast<uint16_t>(analogRead(SENSOR_PINS[i])) & 0x03FFu;
  }
}

void packRaw(const uint16_t raw[SENSOR_COUNT], uint8_t packetOut[RAW_PACKET_SIZE]) {
  uint64_t packed = 0;
  for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
    packed |= (static_cast<uint64_t>(raw[i] & 0x03FFu) << (10u * i));
  }
  for (uint8_t i = 0; i < RAW_PACKET_SIZE; i++) {
    packetOut[i] = static_cast<uint8_t>((packed >> (8u * i)) & 0xFFu);
  }
}

uint8_t checksumRaw(const uint8_t packet[RAW_PACKET_SIZE]) {
  uint8_t crc = 0;
  for (uint8_t i = 0; i < RAW_PACKET_SIZE; i++) {
    crc ^= packet[i];
  }
  return crc;
}

void writeLatestRawPacket() {
  uint8_t packet[RAW_PACKET_SIZE];
  packRaw(latestRaw, packet);
  softSerial.write(FRAME_SYNC_0);
  softSerial.write(FRAME_SYNC_1);
  softSerial.write(packet, RAW_PACKET_SIZE);
  softSerial.write(checksumRaw(packet));
  packetsSentCount++;
}

void maybeSampleSensors() {
  const unsigned long nowUs = micros();
  if (nowUs - lastSampleUs < SAMPLE_INTERVAL_US) return;
  lastSampleUs = nowUs;
  sampleRaw(latestRaw);
}

void handleCommand(char cmd) {
  if (cmd == CMD_READ_ONCE) {
    sampleRaw(latestRaw);  // always serve freshest reading for deterministic request mode
    writeLatestRawPacket();
    cmdReadOnceCount++;
  } else if (cmd == CMD_STREAM_ON) {
    streamEnabled = true;
    cmdStreamOnCount++;
  } else if (cmd == CMD_STREAM_OFF) {
    streamEnabled = false;
    cmdStreamOffCount++;
  } else if (cmd == '\n' || cmd == '\r' || cmd == ' ' || cmd == '\t') {
    // ignore
  } else {
    ignoredCmdCount++;
  }
}

void maybeStream() {
  if (!streamEnabled) return;

  const unsigned long nowMs = millis();
  if (nowMs - lastStreamMs < STREAM_INTERVAL_MS) return;
  lastStreamMs = nowMs;
  writeLatestRawPacket();
}

void maybePrintDebug() {
  if (!DEBUG_SERIAL) return;
  const unsigned long nowMs = millis();
  if (nowMs - lastDebugMs < DEBUG_PERIOD_MS) return;
  lastDebugMs = nowMs;

  Serial.print(F("line_reader,stream="));
  Serial.print(streamEnabled ? 1 : 0);
  Serial.print(F(",cmdR="));
  Serial.print(cmdReadOnceCount);
  Serial.print(F(",cmdS="));
  Serial.print(cmdStreamOnCount);
  Serial.print(F(",cmdP="));
  Serial.print(cmdStreamOffCount);
  Serial.print(F(",pkt="));
  Serial.print(packetsSentCount);
  Serial.print(F(",badCmd="));
  Serial.print(ignoredCmdCount);
  Serial.print(F(",raw="));
  for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
    Serial.print(latestRaw[i]);
    if (i + 1 < SENSOR_COUNT) Serial.print(',');
  }
  Serial.println();
}

void setup() {
  if (DEBUG_SERIAL) {
    Serial.begin(115200);
  }

  pinMode(IR_EMITTER_PIN, OUTPUT);
  digitalWrite(IR_EMITTER_PIN, HIGH);

  softSerial.begin(SOFTSER_BAUD);

  for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
    pinMode(SENSOR_PINS[i], INPUT);
  }

  sampleRaw(latestRaw);
  writeLatestRawPacket();

  if (DEBUG_SERIAL) {
    Serial.println(F("line_reader ready"));
    Serial.print(F("pins A0..A5, UART RX="));
    Serial.print(SOFTSER_RX_PIN);
    Serial.print(F(" TX="));
    Serial.print(SOFTSER_TX_PIN);
    Serial.print(F(" baud="));
    Serial.println(SOFTSER_BAUD);
  }
}

void loop() {
  softSerial.listen();
  maybeSampleSensors();

  while (softSerial.available() > 0) {
    const char cmd = static_cast<char>(softSerial.read());
    handleCommand(cmd);
  }

  maybeStream();
  maybePrintDebug();
}
