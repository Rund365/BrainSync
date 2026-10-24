// Protocol 2: ESP-NOW messages (sender -> receiver) and the text lines the receiver prints for the PC.
// Documentation/IMU_Integration_Plan.md §9 and §11. The PC side is server/utils/hardware/sensor_stream.py.
// Keep identical to the copy in the other sketch folder (a unit test checks this).
#pragma once
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>

#define PROTOCOL_VERSION 2
#define FIRMWARE_VERSION 2
#define IMU_BATCH_N 5        // samples per IMU message (50 ms at 100 Hz)
#define LEGACY_PACKET_LEN 6  // size of the protocol-1 Packet, to recognize an old sender

enum MsgType : uint8_t {
  MSG_STEP = 1,
  MSG_BPM_DELTA = 2,
  MSG_CMD = 3,  // both directions
  MSG_IMU_BATCH = 4,
  MSG_STATUS = 5,
  MSG_TIME_PONG = 6,
  MSG_LIFT = 7,
};

enum CmdCode : uint8_t {
  CMD_CAL_WEIGHT = 1,  // arg: margin; reply arg: new threshold, -1 = error
  CMD_IMU_STREAM = 2,  // arg: 0 off, 1 on; reply arg: the new state
  CMD_TIME_PING = 3,   // arg: ping id; reply is MSG_TIME_PONG
};

// Every message starts with this header. seq counts per message type (loss detection),
// t_us is the sender's esp_timer_get_time(), lower 32 bits.
typedef struct __attribute__((packed)) {
  uint8_t type;
  uint8_t version;
  uint16_t seq;
  uint32_t t_us;
} MsgHeader;

typedef struct __attribute__((packed)) {
  MsgHeader h;  // t_us: FSR threshold crossing
  uint32_t interval_ms;
  uint8_t foot;  // 1 = right, 2 = left
  uint16_t fsr;
} MsgStep;

typedef struct __attribute__((packed)) {
  MsgHeader h;  // t_us: FSR release
  uint8_t foot;
  uint16_t fsr;
} MsgLift;

typedef struct __attribute__((packed)) {
  MsgHeader h;
  int8_t delta;
} MsgBpmDelta;

typedef struct __attribute__((packed)) {
  MsgHeader h;
  uint8_t cmd;
  int16_t arg;
} MsgCmd;

typedef struct __attribute__((packed)) {
  int16_t ax, ay, az;  // raw LSB, 100 per m/s²
  int16_t gx, gy, gz;  // raw LSB, 16 per dps
} ImuRaw;

typedef struct __attribute__((packed)) {
  uint32_t t_us;  // taken just before reading the sensors
  uint8_t valid;  // bit0 left, bit1 right
  ImuRaw left;
  ImuRaw right;
} ImuSample;

typedef struct __attribute__((packed)) {
  MsgHeader h;  // t_us: first sample
  uint8_t n;
  ImuSample s[IMU_BATCH_N];
} MsgImuBatch;

typedef struct __attribute__((packed)) {
  MsgHeader h;
  uint32_t i2c_err_left, i2c_err_right;  // samples lost to bus errors since boot
  uint8_t sys_left, sys_right;           // BNO055 SYS_STATUS, 255 = no answer
  uint8_t fw;
} MsgStatus;

typedef struct __attribute__((packed)) {
  MsgHeader h;  // t_us: when the ping arrived
  uint16_t ping_id;
} MsgPong;

static_assert(sizeof(MsgHeader) == 8, "header layout");
static_assert(sizeof(MsgImuBatch) <= 250, "ESP-NOW payload limit");

inline void fillHeader(MsgHeader &h, uint8_t type, uint16_t seq, uint32_t t_us) {
  h.type = type;
  h.version = PROTOCOL_VERSION;
  h.seq = seq;
  h.t_us = t_us;
}

inline size_t messageLength(uint8_t type) {
  switch (type) {
    case MSG_STEP: return sizeof(MsgStep);
    case MSG_BPM_DELTA: return sizeof(MsgBpmDelta);
    case MSG_CMD: return sizeof(MsgCmd);
    case MSG_IMU_BATCH: return sizeof(MsgImuBatch);
    case MSG_STATUS: return sizeof(MsgStatus);
    case MSG_TIME_PONG: return sizeof(MsgPong);
    case MSG_LIFT: return sizeof(MsgLift);
    default: return 0;
  }
}

enum PacketCheck : uint8_t { PKT_OK, PKT_BAD_LENGTH, PKT_BAD_VERSION };

inline PacketCheck checkPacket(const uint8_t *data, int len) {
  if (len < (int)sizeof(MsgHeader)) return PKT_BAD_LENGTH;
  if (data[1] != PROTOCOL_VERSION) return PKT_BAD_VERSION;
  size_t want = messageLength(data[0]);
  if (want == 0 || (size_t)len != want) return PKT_BAD_LENGTH;
  return PKT_OK;
}

// ---------------- Serial lines (receiver -> PC) ----------------
inline int formatStepLine(char *out, size_t cap, const MsgStep &m, uint32_t interval_ms,
                          float instant_bpm, float avg_bpm) {
  return snprintf(out, cap, "STEP,%u,%lu,%u,%lu,%u,%.2f,%.2f", (unsigned)m.h.seq, (unsigned long)m.h.t_us,
                  (unsigned)m.foot, (unsigned long)interval_ms, (unsigned)m.fsr, instant_bpm, avg_bpm);
}

inline int formatImuLine(char *out, size_t cap, uint16_t seq, const ImuSample &s) {
  return snprintf(out, cap, "IMU,%u,%lu,%u,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d", (unsigned)seq,
                  (unsigned long)s.t_us, (unsigned)s.valid,
                  s.left.ax, s.left.ay, s.left.az, s.left.gx, s.left.gy, s.left.gz,
                  s.right.ax, s.right.ay, s.right.az, s.right.gx, s.right.gy, s.right.gz);
}

inline int formatLiftLine(char *out, size_t cap, const MsgLift &m) {
  return snprintf(out, cap, "LIFT,%u,%lu,%u,%u", (unsigned)m.h.seq, (unsigned long)m.h.t_us,
                  (unsigned)m.foot, (unsigned)m.fsr);
}

inline int formatStatusLine(char *out, size_t cap, const MsgStatus &m) {
  return snprintf(out, cap, "STATUS,%lu,%lu,%lu,%u,%u,%u", (unsigned long)m.h.t_us,
                  (unsigned long)m.i2c_err_left, (unsigned long)m.i2c_err_right,
                  (unsigned)m.sys_left, (unsigned)m.sys_right, (unsigned)m.fw);
}

inline int formatPongLine(char *out, size_t cap, const MsgPong &m) {
  return snprintf(out, cap, "PONG,%u,%lu", (unsigned)m.ping_id, (unsigned long)m.h.t_us);
}
