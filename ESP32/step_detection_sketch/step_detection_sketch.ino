// Sender ESP32 (on the user): FSR step detection, two BNO055 IMUs, BPM buttons -> ESP-NOW.
// Protocol 2, see protocol_v2.h. The IMUs run in their own task at 100 Hz; loop() keeps the
// FSR detection, the buttons and the commands from the receiver.
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>
#include <esp_timer.h>
#include <Wire.h>
#include "protocol_v2.h"

//setting up the receiver MAC address
uint8_t receiverMac[] = {0xfc, 0xb4, 0x67, 0xf4, 0x58, 0xa8};

// Pins:
const int RIGHT_PIN = 33;
const int LEFT_PIN = 32;
const int DOWN_BUTTON = 25;
const int UP_BUTTON = 26;

// IMU wiring. Two options:
//   IMU_ONE_BUS 1: both IMUs on one I2C bus (current lab wiring): right at 0x28 (ADR to GND),
//                  left at 0x29 (ADR to 3.3V), both on SDA 21 / SCL 22.
//   IMU_ONE_BUS 0: one bus per foot (Integration Plan §6.3): left on 21/22, right on 18/19,
//                  both at 0x28. A broken cable on one foot then cannot disturb the other.
#define IMU_ONE_BUS 1

#if IMU_ONE_BUS
const int LEFT_SDA = 21;
const int LEFT_SCL = 22;
const int RIGHT_SDA = 21;
const int RIGHT_SCL = 22;
const uint8_t LEFT_ADDR = 0x29;   // ADR pin high
const uint8_t RIGHT_ADDR = 0x28;  // ADR pin low
#else
const int LEFT_SDA = 21;
const int LEFT_SCL = 22;
const int RIGHT_SDA = 18;
const int RIGHT_SCL = 19;
const uint8_t LEFT_ADDR = 0x28;   // ADR pin low
const uint8_t RIGHT_ADDR = 0x28;  // ADR pin low
#endif

// Foot ID's:
const uint8_t RIGHT = 1;
const uint8_t LEFT = 2;

// ---------------- BNO055 ----------------
// Register map: Bosch BNO055 datasheet, section 4
const uint8_t BNO055_CHIP_ID = 0xA0;
const uint8_t REG_CHIP_ID = 0x00;
const uint8_t REG_PAGE_ID = 0x07;
const uint8_t REG_ACC_DATA = 0x08;  // 18 bytes: accel x,y,z, mag x,y,z, gyro x,y,z (int16 LE)
const uint8_t REG_SYS_STATUS = 0x39;
const uint8_t REG_UNIT_SEL = 0x3B;
const uint8_t REG_OPR_MODE = 0x3D;
const uint8_t REG_PWR_MODE = 0x3E;
const uint8_t REG_ACC_CONFIG = 0x08;    // page 1
const uint8_t REG_GYR_CONFIG_0 = 0x0A;  // page 1
const uint8_t REG_GYR_CONFIG_1 = 0x0B;  // page 1
const uint8_t MODE_CONFIG = 0x00;
const uint8_t MODE_ACCGYRO = 0x05;  // raw accel + gyro, no fusion: ranges are ours to set
const uint8_t ACC_16G_125HZ = 0x13;  // range ±16 g (heel strikes clip at ±4 g), bandwidth 125 Hz
const uint8_t GYR_2000DPS_116HZ = 0x10;
const uint8_t UNITS_MS2_DPS = 0x00;  // 1 m/s² = 100 LSB, 1 dps = 16 LSB

const uint32_t I2C_HZ = 100000;  // long cables: try 50000 if bus errors show up
const uint16_t I2C_TIMEOUT_MS = 20;
const uint32_t IMU_PERIOD_US = 10000;  // 100 Hz
const uint32_t STATUS_EVERY = 100;     // samples, = once per second
const uint8_t IMU_MAX_FAILS = 10;      // consecutive failed reads before re-initializing
const uint32_t IMU_RETRY_MS = 2000;

class ImuFoot {
  public:
    TwoWire &bus;
    int sda, scl;
    uint8_t addr;  // I2C address of this foot's BNO055
    bool ok = false;
    uint8_t failStreak = 0;
    uint32_t errors = 0;
    uint8_t sys = 255;
    uint32_t nextRetryMs = 0;
    ImuFoot(TwoWire &b, int sdaPin, int sclPin, uint8_t address)
      : bus(b), sda(sdaPin), scl(sclPin), addr(address) {}
};

#if IMU_ONE_BUS
ImuFoot imuLeft(Wire, LEFT_SDA, LEFT_SCL, LEFT_ADDR);
ImuFoot imuRight(Wire, RIGHT_SDA, RIGHT_SCL, RIGHT_ADDR);  // same bus as the left foot
#else
ImuFoot imuLeft(Wire, LEFT_SDA, LEFT_SCL, LEFT_ADDR);
ImuFoot imuRight(Wire1, RIGHT_SDA, RIGHT_SCL, RIGHT_ADDR);
#endif
TaskHandle_t imuTaskHandle = nullptr;
volatile bool imuStreaming = true;

// Step detection tuning (mutable after calibration):
// Per-sensor thresholds and baselines
int thresholdRight = 1200;
int thresholdLeft = 1200;
int baselineRight = 0;   // Resting state value
int baselineLeft = 0;    // Resting state value

// Pressure buffer as percentage of threshold (hysteresis)
const float PRESSURE_BUFFER_PERCENT = 0.55;  // 55% of threshold
int pressureBufferRight = 540;  // Will be recalculated in calibration
int pressureBufferLeft = 540;   // Will be recalculated in calibration

const uint32_t REFRACTORY_MS = 120;

uint32_t lastStepTime = 0;
uint32_t lastStepSentTime = 0;
bool leftFlag = false;
bool rightFlag = false;

// Sequence numbers, one per message type; each type is sent from one task only
uint16_t seqStep = 0, seqLift = 0, seqBtn = 0, seqCmd = 0, seqPong = 0;  // loop()
uint16_t seqImu = 0, seqStatus = 0;                                       // IMU task

// Commands from the receiver, handed from the ESP-NOW callback to loop()
struct PendingCmd {
  MsgCmd msg;
  uint32_t rx_us;
};
QueueHandle_t cmdQueue;

// Button tuning (hold-repeat):
const uint32_t DEBOUNCE_MS = 25;
const uint32_t FIRST_REPEAT_DELAY_MS = 200;
const uint32_t REPEAT_RATE_MS = 60;

// Button state (minimal, robust)
struct Button {
  int pin;
  bool stable = HIGH;     // debounced state
  bool lastRaw = HIGH;    // last raw read
  uint32_t tChange = 0;   // last raw change time
  uint32_t tPress = 0;    // 0 = not pressed, else press start time
  uint32_t tRepeat = 0;   // last repeat tick time
  Button() = default;
  explicit Button(int p) : pin(p) {}
};

Button buttonUp(UP_BUTTON);
Button buttonDown(DOWN_BUTTON);

// The callback's first argument changed in ESP32 Arduino core 3
#if ESP_ARDUINO_VERSION_MAJOR >= 3
#define RECV_CB_ARGS const esp_now_recv_info_t *, const uint8_t *incomingData, int len
#else
#define RECV_CB_ARGS const uint8_t *, const uint8_t *incomingData, int len
#endif

// ---------------- Helpers ----------------
inline uint32_t nowUs() {
  return (uint32_t)esp_timer_get_time();
}

inline void sendMsg(const void *msg, size_t len) {
  esp_now_send(receiverMac, (const uint8_t *)msg, len);
}

// ESP-NOW receive callback to catch control commands
void onDataRecv(RECV_CB_ARGS) {
  uint32_t rx_us = nowUs();
  if (checkPacket(incomingData, len) != PKT_OK || incomingData[0] != MSG_CMD) return;
  PendingCmd p;
  memcpy(&p.msg, incomingData, sizeof(MsgCmd));
  p.rx_us = rx_us;
  xQueueSend(cmdQueue, &p, 0);
}

// Calibration helper: sample both FSRs, set PER-SENSOR thresholds
int calibrateWeight(uint16_t samples = 200, uint16_t delayMs = 5, int margin = 150, int minThresh = 300) {
  long sumR = 0, sumL = 0;

  Serial.println("Calibrating... Stand normally on both sensors.");

  for (uint16_t i = 0; i < samples; i++) {
    sumR += analogRead(RIGHT_PIN);
    sumL += analogRead(LEFT_PIN);
    delay(delayMs);
  }

  int avgR = sumR / samples;
  int avgL = sumL / samples;

  // Store baselines (resting/standing state)
  baselineRight = avgR;
  baselineLeft = avgL;

  // Calculate individual thresholds: baseline - margin
  // This accounts for weaker sensors automatically
  thresholdRight = max(minThresh, avgR - margin);
  thresholdLeft = max(minThresh, avgL - margin);

  // Calculate pressure buffers as PERCENTAGE of threshold
  pressureBufferRight = (int)(thresholdRight * PRESSURE_BUFFER_PERCENT);
  pressureBufferLeft = (int)(thresholdLeft * PRESSURE_BUFFER_PERCENT);

  // Safety check: minimum buffer
  pressureBufferRight = max(200, pressureBufferRight);
  pressureBufferLeft = max(200, pressureBufferLeft);

  Serial.print("Right sensor - baseline: ");
  Serial.print(avgR);
  Serial.print(", threshold: ");
  Serial.print(thresholdRight);
  Serial.print(", buffer: ");
  Serial.println(pressureBufferRight);

  Serial.print("Left sensor - baseline: ");
  Serial.print(avgL);
  Serial.print(", threshold: ");
  Serial.print(thresholdLeft);
  Serial.print(", buffer: ");
  Serial.println(pressureBufferLeft);

  // Return average threshold for ACK
  return (thresholdRight + thresholdLeft) / 2;
}

// Send a control ACK/ERR back to receiver (arg can carry status or threshold)
void sendControlAck(uint8_t cmd, int16_t arg) {
  MsgCmd resp{};
  fillHeader(resp.h, MSG_CMD, seqCmd++, nowUs());
  resp.cmd = cmd;
  resp.arg = arg;
  sendMsg(&resp, sizeof(resp));
}

// Process pending control commands
void processControlIfPending() {
  PendingCmd p;
  while (xQueueReceive(cmdQueue, &p, 0) == pdTRUE) {
    if (p.msg.cmd == CMD_CAL_WEIGHT) {
      int margin = (int)p.msg.arg;
      if (margin <= 0) margin = 150;
      int newThresh = calibrateWeight(200, 5, margin);
      sendControlAck(CMD_CAL_WEIGHT, (int16_t)newThresh);  // Ack with new threshold value
    } else if (p.msg.cmd == CMD_IMU_STREAM) {
      imuStreaming = p.msg.arg != 0;
      sendControlAck(CMD_IMU_STREAM, imuStreaming ? 1 : 0);
    } else if (p.msg.cmd == CMD_TIME_PING) {
      MsgPong pong{};
      fillHeader(pong.h, MSG_TIME_PONG, seqPong++, p.rx_us);
      pong.ping_id = (uint16_t)p.msg.arg;
      sendMsg(&pong, sizeof(pong));
    } else {
      sendControlAck(p.msg.cmd, (int16_t)-1); // Unknown command -> error
    }
  }
}

// Fast time-based low-pass filter (IIR). 1 ADC read per loop.
int lowPass(int raw, int &state) {
  // 0.75*old + 0.25*new
  state = (state * 3 + raw) / 4;
  return state;
}

// Debounced pressed? (INPUT_PULLUP => LOW = pressed)
bool debouncedPressed(Button &b, uint32_t now) {
  bool raw = digitalRead(b.pin);
  if (raw != b.lastRaw) { b.lastRaw = raw; b.tChange = now; }
  if (now - b.tChange >= DEBOUNCE_MS) b.stable = raw;
  return (b.stable == LOW);
}

// BPM acceleration while holding (keeps network rate low but feels fast)
int8_t accelStep(uint32_t heldMs) {
  if (heldMs >= 2500) return 5;
  if (heldMs >= 1000) return 2;
  return 1;
}

void sendBpmDelta(int8_t delta) {
  MsgBpmDelta m{};
  fillHeader(m.h, MSG_BPM_DELTA, seqBtn++, nowUs());
  m.delta = delta;
  sendMsg(&m, sizeof(m));
}

void handleButton(Button &b, int8_t dir, uint32_t now) {
  bool pressed = debouncedPressed(b, now);

  // PRESS edge: immediate tick
  if (pressed && b.tPress == 0) {
    b.tPress = now;
    b.tRepeat = now;
    sendBpmDelta(dir * 1);
    return;
  }

  // RELEASE edge
  if (!pressed && b.tPress != 0) {
    b.tPress = 0;
    return;
  }

  // HELD: rate-limited ticks
  if (pressed && b.tPress != 0) {
    uint32_t held = now - b.tPress;

    if (held >= FIRST_REPEAT_DELAY_MS && (now - b.tRepeat) >= REPEAT_RATE_MS) {
      sendBpmDelta(dir * accelStep(held));
      b.tRepeat = now;
    }
  }
}

void sendLift(uint8_t foot, int fsr, uint32_t t_us) {
  MsgLift m{};
  fillHeader(m.h, MSG_LIFT, seqLift++, t_us);
  m.foot = foot;
  m.fsr = (uint16_t)fsr;
  sendMsg(&m, sizeof(m));
}

// ---------------- IMU (own task, owns both I2C buses) ----------------
bool bnoWrite(ImuFoot &f, uint8_t reg, uint8_t value) {
  TwoWire &bus = f.bus;
  bus.beginTransmission(f.addr);
  bus.write(reg);
  bus.write(value);
  return bus.endTransmission() == 0;
}

bool bnoRead(ImuFoot &f, uint8_t reg, uint8_t *buf, uint8_t n) {
  TwoWire &bus = f.bus;
  bus.beginTransmission(f.addr);
  bus.write(reg);
  if (bus.endTransmission(false) != 0) return false;
  if (bus.requestFrom(f.addr, n) != n) return false;
  for (uint8_t i = 0; i < n; i++) buf[i] = bus.read();
  return true;
}

bool bnoConfigure(ImuFoot &f) {
  uint8_t id = 0, mode = 0;
  if (!bnoRead(f, REG_CHIP_ID, &id, 1) || id != BNO055_CHIP_ID) return false;
  if (!bnoWrite(f, REG_OPR_MODE, MODE_CONFIG)) return false;
  delay(25);
  bool ok = bnoWrite(f, REG_PAGE_ID, 1)
            && bnoWrite(f, REG_ACC_CONFIG, ACC_16G_125HZ)
            && bnoWrite(f, REG_GYR_CONFIG_0, GYR_2000DPS_116HZ)
            && bnoWrite(f, REG_GYR_CONFIG_1, 0x00)  // normal power
            && bnoWrite(f, REG_PAGE_ID, 0)
            && bnoWrite(f, REG_UNIT_SEL, UNITS_MS2_DPS)
            && bnoWrite(f, REG_PWR_MODE, 0x00)  // normal power
            && bnoWrite(f, REG_OPR_MODE, MODE_ACCGYRO);
  if (!ok) return false;
  delay(20);
  return bnoRead(f, REG_OPR_MODE, &mode, 1) && mode == MODE_ACCGYRO;
}

void busStart(ImuFoot &f) {
  f.bus.begin(f.sda, f.scl, I2C_HZ);
  f.bus.setTimeOut(I2C_TIMEOUT_MS);
}

// Frees a bus whose sensor holds SDA low mid-transfer: clock it out, then start over
void busRecover(ImuFoot &f) {
  f.bus.end();
  pinMode(f.sda, INPUT_PULLUP);
  pinMode(f.scl, OUTPUT_OPEN_DRAIN);
  for (int i = 0; i < 9; i++) {
    digitalWrite(f.scl, LOW);
    delayMicroseconds(5);
    digitalWrite(f.scl, HIGH);
    delayMicroseconds(5);
  }
  busStart(f);
}

bool imuRead(ImuFoot &f, ImuRaw &out) {
  uint8_t b[18];
  if (!f.ok || !bnoRead(f, REG_ACC_DATA, b, sizeof(b))) {
    f.errors++;
    if (f.ok && ++f.failStreak >= IMU_MAX_FAILS) {
      f.ok = false;
      f.nextRetryMs = millis();
    }
    return false;
  }
  f.failStreak = 0;
  out.ax = (int16_t)(b[0] | (b[1] << 8));
  out.ay = (int16_t)(b[2] | (b[3] << 8));
  out.az = (int16_t)(b[4] | (b[5] << 8));
  out.gx = (int16_t)(b[12] | (b[13] << 8));
  out.gy = (int16_t)(b[14] | (b[15] << 8));
  out.gz = (int16_t)(b[16] | (b[17] << 8));
  return true;
}

// Runs between samples, never between a sample's timestamp and its read
void imuRecoverIfDue(ImuFoot &f) {
  uint32_t now = millis();
  if (f.ok || (int32_t)(now - f.nextRetryMs) < 0) return;
  f.nextRetryMs = now + IMU_RETRY_MS;
  busRecover(f);
  f.ok = bnoConfigure(f);
  f.failStreak = 0;
}

void readSysStatus(ImuFoot &f) {
  uint8_t sys = 255;
  if (!f.ok || !bnoRead(f, REG_SYS_STATUS, &sys, 1)) sys = 255;
  f.sys = sys;
}

void sendStatus() {
  readSysStatus(imuLeft);
  readSysStatus(imuRight);
  MsgStatus m{};
  fillHeader(m.h, MSG_STATUS, seqStatus++, nowUs());
  m.i2c_err_left = imuLeft.errors;
  m.i2c_err_right = imuRight.errors;
  m.sys_left = imuLeft.sys;
  m.sys_right = imuRight.sys;
  m.fw = FIRMWARE_VERSION;
  sendMsg(&m, sizeof(m));
}

void imuTick(void *) {
  xTaskNotifyGive(imuTaskHandle);
}

void imuTask(void *) {
  busStart(imuLeft);
  if (&imuRight.bus != &imuLeft.bus) busStart(imuRight);  // one-bus wiring: already started
  delay(700);  // BNO055 boot time after power-on
  imuLeft.ok = bnoConfigure(imuLeft);
  imuRight.ok = bnoConfigure(imuRight);
  Serial.printf("IMU left %s, right %s\n", imuLeft.ok ? "ok" : "MISSING", imuRight.ok ? "ok" : "MISSING");

  esp_timer_create_args_t timerArgs = {};
  timerArgs.callback = imuTick;
  timerArgs.name = "imu";
  esp_timer_handle_t timer;
  esp_timer_create(&timerArgs, &timer);
  esp_timer_start_periodic(timer, IMU_PERIOD_US);

  MsgImuBatch batch{};
  uint8_t n = 0;
  uint32_t samples = 0;
  for (;;) {
    // Ticks missed during a slow read or a recovery are dropped, not caught up in a burst
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    ImuSample &s = batch.s[n];
    memset(&s, 0, sizeof(s));
    s.t_us = nowUs();
    if (imuRead(imuLeft, s.left)) s.valid |= 1;
    if (imuRead(imuRight, s.right)) s.valid |= 2;

    if (++n == IMU_BATCH_N) {
      if (imuStreaming) {
        fillHeader(batch.h, MSG_IMU_BATCH, seqImu++, batch.s[0].t_us);
        batch.n = n;
        sendMsg(&batch, sizeof(batch));
      }
      n = 0;
    }

    if (++samples % STATUS_EVERY == 0) sendStatus();
    imuRecoverIfDue(imuLeft);
    imuRecoverIfDue(imuRight);
  }
}

// ---------------- Setup ----------------
void setup() {
  Serial.begin(115200);

  // FSR analog inputs:
  pinMode(RIGHT_PIN, INPUT);
  pinMode(LEFT_PIN, INPUT);

  // Buttons:
  pinMode(DOWN_BUTTON, INPUT_PULLUP);
  pinMode(UP_BUTTON, INPUT_PULLUP);

  cmdQueue = xQueueCreate(4, sizeof(PendingCmd));

  WiFi.mode(WIFI_STA);

  //initializing esp_now:
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing esp_now");
    return;
  }

  esp_now_peer_info_t peerInfo = {};

  memcpy(peerInfo.peer_addr,receiverMac, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }

  esp_now_register_recv_cb(onDataRecv);

  // Above loop() (priority 1) on the same core: I2C waits let the FSR loop run meanwhile
  xTaskCreatePinnedToCore(imuTask, "imu", 4096, nullptr, 3, &imuTaskHandle, 1);

  // Initial calibration on startup
  Serial.println("Starting initial calibration in 2 seconds...");
  Serial.println("Please stand normally on BOTH sensors.");
  delay(2000);
  calibrateWeight(200, 5, 200);  // Increased margin to 200 for less sensitivity

  Serial.println("\nCalibration complete. Ready to send.");
}

// ---------------- Loop ----------------
void loop() {
  // Getting the current time:
  uint32_t now =  millis();
  uint32_t tUs = nowUs();

  // Handle inbound control commands (e.g., calibration)
  processControlIfPending();

  // Buttons:
  bool upRaw = (digitalRead(UP_BUTTON) == LOW);
  bool downRaw = (digitalRead(DOWN_BUTTON) == LOW);
  // Safety: if both pressed, ignore
  if (!(upRaw && downRaw)) {
    handleButton(buttonUp, +1, now);
    handleButton(buttonDown, -1, now);
  }

  // FSR read + filtering:
  static int rState = 0, lState = 0;
  int rightFsr = lowPass(analogRead(RIGHT_PIN), rState);
  int leftFsr  = lowPass(analogRead(LEFT_PIN),  lState);

  // Flag reset (per-sensor thresholds with percentage-based buffers) = foot lifted:
  if (rightFlag && rightFsr <= thresholdRight - pressureBufferRight) {
    rightFlag = false;
    sendLift(RIGHT, rightFsr, tUs);
  }
  if (leftFlag && leftFsr <= thresholdLeft - pressureBufferLeft) {
    leftFlag = false;
    sendLift(LEFT, leftFsr, tUs);
  }

  // New step detection:
  bool stepDetected = false;
  uint8_t stepFootId = 0;
  int triggeredFsr = 0;

  // Checking if there is pressure on the sensors (per-sensor thresholds)
  if(!rightFlag && rightFsr > thresholdRight){
    rightFlag = true;
    stepDetected = true;
    stepFootId = RIGHT;
    triggeredFsr = rightFsr;
  }
  if(!leftFlag && leftFsr > thresholdLeft){
    leftFlag = true;
    stepDetected = true;
    stepFootId = LEFT;
    triggeredFsr = leftFsr;
  }

  // Refractory guard + send
  if (stepDetected && (now - lastStepSentTime) >= REFRACTORY_MS) {
    lastStepSentTime = now;

    MsgStep p{};
    fillHeader(p.h, MSG_STEP, seqStep++, tUs);
    p.interval_ms = now - lastStepTime;
    lastStepTime = now;
    p.foot = stepFootId;
    p.fsr = (uint16_t)triggeredFsr;
    sendMsg(&p, sizeof(p));
  }
}
