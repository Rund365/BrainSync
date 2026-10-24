// Receiver ESP32 (at the PC): ESP-NOW messages from the sender -> text lines on the serial port.
// Protocol 2, see protocol_v2.h. Sensor lines are only printed during a session (START until
// the next RESET), so the PC's command handshake reads only answers.
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>
#include "protocol_v2.h"

const uint32_t BAUD = 921600;

// ==== SETTINGS ====
int smoothingWindow = 3;  // Steps to average for BPM (changeable via serial)
int updateStride = 1;     // Update BPM every N steps (changeable via serial)
bool sessionActive = false;
bool sessionStarted = false;  // true until the first step of the session
bool sessionStarting = true;
int stepCounter = 0;
float lastCalculatedBPM = 0.0;

// Sender MAC (ESP32 on foot sensors).
uint8_t senderMac[] = {0xfc, 0xb4, 0x67, 0xf4, 0x5f, 0x68};

// ---------------- CIRCULAR BUFFER CLASS ----------------
#define MAX_WINDOW_SIZE 20

class StepBuffer {
  private:
    uint32_t intervals[MAX_WINDOW_SIZE];
    int count = 0;
    int head = 0;

  public:
    void add(uint32_t intervalMS) {
      intervals[head] = intervalMS;
      head = (head + 1) % MAX_WINDOW_SIZE;
      if (count < MAX_WINDOW_SIZE) count++;
    }

    // Calculate BPM of the last 'n' steps
    float getAverageBPM(int n) {
      int effectiveN = (n < count) ? n : count;
      if (effectiveN == 0) return 0.0;

      uint32_t totalInterval = 0;
      int currentIdx = head;  // newest item is at (head - 1)
      for (int i = 0; i < effectiveN; i++) {
        currentIdx = (currentIdx - 1 + MAX_WINDOW_SIZE) % MAX_WINDOW_SIZE;
        totalInterval += intervals[currentIdx];
      }

      float avg = (float)totalInterval / effectiveN;
      if (avg == 0) return 0.0;
      return 60000.0 / avg;
    }

    void clear() {
      count = 0;
      head = 0;
    }

    // Resize "virtual" window (cannot exceed MAX_WINDOW_SIZE)
    void trimToSize(int newSize) {
      if (newSize < count) count = newSize;
    }
};

StepBuffer stepHistory;

// ---------------- RECEIVE QUEUE ----------------
// The ESP-NOW callback only copies the bytes; loop() parses and prints them.
struct RxPacket {
  uint8_t len;
  uint8_t data[250];
};
const int RX_QUEUE_DEPTH = 32;
QueueHandle_t rxQueue;

// Link counters, reported in RXSTAT once per second during a session
volatile uint32_t rxCount = 0;
volatile uint32_t rxOverflows = 0;
uint32_t badLength = 0;
uint32_t badVersion = 0;
bool versionErrorReported = false;
unsigned long nextRxStat = 0;

uint16_t cmdSeq = 0;

// Non-blocking LED blink on each step
const unsigned long LED_MS = 50;
unsigned long ledOffAt = 0;
bool ledOn = false;

char line[200];

// ---------------- ESP-NOW CALLBACK ----------------
// The callback's first argument changed in ESP32 Arduino core 3
#if ESP_ARDUINO_VERSION_MAJOR >= 3
#define RECV_CB_ARGS const esp_now_recv_info_t *, const uint8_t *incomingData, int len
#else
#define RECV_CB_ARGS const uint8_t *, const uint8_t *incomingData, int len
#endif

void onDataRecv(RECV_CB_ARGS) {
  rxCount++;
  if (len <= 0 || len > (int)sizeof(RxPacket::data)) return;
  RxPacket p;
  p.len = (uint8_t)len;
  memcpy(p.data, incomingData, len);
  if (xQueueSend(rxQueue, &p, 0) != pdTRUE) rxOverflows++;
}

// ---------------- SETUP ----------------
void setup() {
  Serial.begin(BAUD);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  rxQueue = xQueueCreate(RX_QUEUE_DEPTH, sizeof(RxPacket));

  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // The sender must be a peer, or commands (calibration, ping) cannot be sent to it
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, senderMac, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add sender as peer");
  }

  esp_now_register_recv_cb(onDataRecv);
}

// ---------------- CONTROL SENDER ----------------
bool sendControl(uint8_t cmd, int16_t arg) {
  MsgCmd m{};
  fillHeader(m.h, MSG_CMD, cmdSeq++, (uint32_t)micros());
  m.cmd = cmd;
  m.arg = arg;
  return esp_now_send(senderMac, (const uint8_t *)&m, sizeof(m)) == ESP_OK;
}

// ---------------- SERIAL COMMAND HANDLERS ----------------
int commandArg(const String &command, int fallback) {
  int commaIndex = command.indexOf(',');
  if (commaIndex == -1) return fallback;
  return command.substring(commaIndex + 1).toInt();
}

void handleReset() {
  sessionStarting = true;
  sessionActive = false;
  Serial.println("ACK,RESET");
}

void handleStart(unsigned long now) {
  if (!sessionStarting) return;
  stepHistory.clear();
  sessionStarting = false;
  sessionStarted = true;
  sessionActive = true;
  stepCounter = 0;
  lastCalculatedBPM = 0.0;
  nextRxStat = now + 1000;
  Serial.println("ACK,START");
}

void handleSetWindow(const String &command) {
  int val = commandArg(command, -1);
  if (val > 0 && val <= MAX_WINDOW_SIZE) {
    smoothingWindow = val;
    if (updateStride > smoothingWindow) updateStride = smoothingWindow;
    stepHistory.trimToSize(smoothingWindow);
    Serial.print("ACK,WINDOW,");
    Serial.println(smoothingWindow);
  }
}

void handleSetStride(const String &command) {
  int val = commandArg(command, -1);
  if (val > 0 && val <= smoothingWindow) {
    updateStride = val;
    Serial.print("ACK,STRIDE,");
    Serial.println(updateStride);
  }
}

void handleCalibrateWeight(const String &command) {
  int margin = commandArg(command, 150);
  if (margin <= 0) margin = 150;
  bool ok = sendControl(CMD_CAL_WEIGHT, (int16_t)margin);
  Serial.println(ok ? "ACK,CAL_WEIGHT" : "ERR,CAL_WEIGHT");
  // Request was dispatched to the sender; its threshold arrives later as ACK,CAL_WEIGHT,<value>
  Serial.println("ACK,CAL_WEIGHT,DISPATCHED");
}

void handleImuStream(const String &command) {
  int val = commandArg(command, -1);
  if (val != 0 && val != 1) return;
  if (!sendControl(CMD_IMU_STREAM, (int16_t)val)) Serial.println("ERR,IMU_STREAM");
}

void handlePing(const String &command) {
  int id = commandArg(command, -1);
  if (id < 0) return;
  if (!sendControl(CMD_TIME_PING, (int16_t)id)) Serial.println("ERR,PING");
}

void handleCommand(const String &command, unsigned long now) {
  if (command == "RESET") {
    handleReset();
  } else if (command == "START") {
    handleStart(now);
  } else if (command == "VERSION") {
    Serial.print("ACK,VERSION,");
    Serial.println(PROTOCOL_VERSION);
  } else if (command.startsWith("SET_WINDOW,")) {
    handleSetWindow(command);
  } else if (command.startsWith("SET_STRIDE,")) {
    handleSetStride(command);
  } else if (command.startsWith("CAL_WEIGHT")) {
    handleCalibrateWeight(command);
  } else if (command.startsWith("IMU_STREAM,")) {
    handleImuStream(command);
  } else if (command.startsWith("PING,")) {
    handlePing(command);
  }
}

// Reads whatever has arrived without waiting, so a slow line never holds up the radio queue
void processSerialCommands(unsigned long now) {
  static String pending;
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n') {
      pending.trim();
      if (pending.length()) handleCommand(pending, now);
      pending = "";
    } else if (pending.length() < 64) {
      pending += c;
    }
  }
}

// ---------------- MESSAGE HANDLERS ----------------
void handleStep(const MsgStep &m, unsigned long now) {
  uint32_t interval;
  float bpm;

  if (sessionStarted) {
    // First step after START: signal detection but ignore its timing for BPM
    sessionStarted = false;
    interval = 0;
    bpm = 0.0;
  } else {
    interval = m.interval_ms;
    stepHistory.add(interval);
    stepCounter++;
    if (stepCounter % updateStride == 0) {
      bpm = stepHistory.getAverageBPM(smoothingWindow);
      lastCalculatedBPM = bpm;
    } else {
      bpm = lastCalculatedBPM;
    }
  }
  float instantBPM = interval > 0 ? 60000.0 / interval : 0.0;

  formatStepLine(line, sizeof(line), m, interval, instantBPM, bpm);
  Serial.println(line);

  digitalWrite(LED_BUILTIN, HIGH);
  ledOn = true;
  ledOffAt = now + LED_MS;
}

void handleCmdReply(const MsgCmd &m) {
  if (m.cmd == CMD_CAL_WEIGHT) {
    if (m.arg >= 0) {
      Serial.print("ACK,CAL_WEIGHT,");
      Serial.println((int)m.arg);
    } else {
      Serial.println("ERR,CAL_WEIGHT");
    }
  } else if (m.cmd == CMD_IMU_STREAM) {
    Serial.print("ACK,IMU_STREAM,");
    Serial.println((int)m.arg);
  } else {
    Serial.print("ERR,UNKNOWN_CMD,");
    Serial.println((int)m.cmd);
  }
}

void reportVersionError(int version) {
  if (versionErrorReported) return;
  versionErrorReported = true;
  // The sender runs a different protocol: it needs the matching firmware
  Serial.print("ERR,PROTO_VERSION,");
  Serial.println(version);
}

void handlePacket(const RxPacket &p, unsigned long now) {
  PacketCheck check = checkPacket(p.data, p.len);
  if (check == PKT_BAD_VERSION) {
    badVersion++;
    reportVersionError(p.data[1]);
    return;
  }
  if (check == PKT_BAD_LENGTH) {
    badLength++;
    if (p.len == LEGACY_PACKET_LEN) reportVersionError(1);
    return;
  }

  switch (p.data[0]) {
    case MSG_BPM_DELTA: {
      MsgBpmDelta m;
      memcpy(&m, p.data, sizeof(m));
      Serial.print("BTN,");
      Serial.println((int)m.delta);
      break;
    }
    case MSG_CMD: {
      MsgCmd m;
      memcpy(&m, p.data, sizeof(m));
      handleCmdReply(m);
      break;
    }
    case MSG_TIME_PONG: {
      MsgPong m;
      memcpy(&m, p.data, sizeof(m));
      formatPongLine(line, sizeof(line), m);
      Serial.println(line);
      break;
    }
    case MSG_STEP: {
      if (!sessionActive) break;
      MsgStep m;
      memcpy(&m, p.data, sizeof(m));
      handleStep(m, now);
      break;
    }
    case MSG_LIFT: {
      if (!sessionActive) break;
      MsgLift m;
      memcpy(&m, p.data, sizeof(m));
      formatLiftLine(line, sizeof(line), m);
      Serial.println(line);
      break;
    }
    case MSG_IMU_BATCH: {
      if (!sessionActive) break;
      MsgImuBatch m;
      memcpy(&m, p.data, sizeof(m));
      uint8_t n = m.n <= IMU_BATCH_N ? m.n : IMU_BATCH_N;
      for (uint8_t i = 0; i < n; i++) {
        formatImuLine(line, sizeof(line), m.h.seq, m.s[i]);
        Serial.println(line);
      }
      break;
    }
    case MSG_STATUS: {
      if (!sessionActive) break;
      MsgStatus m;
      memcpy(&m, p.data, sizeof(m));
      formatStatusLine(line, sizeof(line), m);
      Serial.println(line);
      break;
    }
  }
}

// ---------------- LOOP ----------------
void loop() {
  unsigned long now = millis();

  processSerialCommands(now);

  RxPacket p;
  while (xQueueReceive(rxQueue, &p, 0) == pdTRUE) {
    handlePacket(p, now);
  }

  if (ledOn && (long)(now - ledOffAt) >= 0) {
    digitalWrite(LED_BUILTIN, LOW);
    ledOn = false;
  }

  if (sessionActive && (long)(now - nextRxStat) >= 0) {
    nextRxStat = now + 1000;
    snprintf(line, sizeof(line), "RXSTAT,%lu,%lu,%lu,%lu", (unsigned long)rxCount,
             (unsigned long)rxOverflows, (unsigned long)badLength, (unsigned long)badVersion);
    Serial.println(line);
  }
}
