// Builds every protocol-2 message with the firmware's header and prints the serial line the
// receiver would print for it. server/tests/test_imu_pipeline.py compiles this with g++ and
// checks each line with the Python parser. Exit code 1 = a packet check failed.
#include <string.h>
#include "../step_receiver_sketch/protocol_v2.h"

static int failures = 0;

static void expect(bool ok, const char *what) {
  if (!ok) {
    printf("FAIL,%s\n", what);
    failures++;
  }
}

template <typename T>
static void expectValid(const T &m, const char *what) {
  expect(checkPacket((const uint8_t *)&m, sizeof(m)) == PKT_OK, what);
}

int main() {
  char line[200];

  MsgStep step{};
  fillHeader(step.h, MSG_STEP, 812, 4000000000u);
  step.interval_ms = 512;
  step.foot = 1;
  step.fsr = 2210;
  expectValid(step, "step");
  formatStepLine(line, sizeof(line), step, step.interval_ms, 60000.0f / 512, 116.4f);
  puts(line);

  MsgImuBatch batch{};
  fillHeader(batch.h, MSG_IMU_BATCH, 65535, 123460000u);
  batch.n = IMU_BATCH_N;
  for (int i = 0; i < IMU_BATCH_N; i++) {
    ImuSample &s = batch.s[i];
    s.t_us = 123460000u + i * 10000;
    s.valid = i == 2 ? 1 : 3;
    s.left = {(int16_t)(-1570 + i), 12, 981, -32768, 0, 32767};
    s.right = {1, 2, 3, 4, 5, (int16_t)(-6 - i)};
  }
  expectValid(batch, "imu");
  for (int i = 0; i < batch.n; i++) {
    formatImuLine(line, sizeof(line), batch.h.seq, batch.s[i]);
    puts(line);
  }

  MsgLift lift{};
  fillHeader(lift.h, MSG_LIFT, 410, 123459000u);
  lift.foot = 2;
  lift.fsr = 750;
  expectValid(lift, "lift");
  formatLiftLine(line, sizeof(line), lift);
  puts(line);

  MsgStatus status{};
  fillHeader(status.h, MSG_STATUS, 7, 123500000u);
  status.i2c_err_left = 3;
  status.i2c_err_right = 70000;
  status.sys_left = 6;
  status.sys_right = 255;
  status.fw = FIRMWARE_VERSION;
  expectValid(status, "status");
  formatStatusLine(line, sizeof(line), status);
  puts(line);

  MsgPong pong{};
  fillHeader(pong.h, MSG_TIME_PONG, 1, 99u);
  pong.ping_id = 42;
  expectValid(pong, "pong");
  formatPongLine(line, sizeof(line), pong);
  puts(line);

  MsgCmd cmd{};
  fillHeader(cmd.h, MSG_CMD, 0, 0);
  expectValid(cmd, "cmd");
  MsgBpmDelta btn{};
  fillHeader(btn.h, MSG_BPM_DELTA, 0, 0);
  expectValid(btn, "bpm delta");

  // Rejected: wrong version, wrong length, unknown type, an old protocol-1 packet
  MsgStep bad = step;
  bad.h.version = 1;
  expect(checkPacket((const uint8_t *)&bad, sizeof(bad)) == PKT_BAD_VERSION, "bad version");
  expect(checkPacket((const uint8_t *)&step, sizeof(step) - 1) == PKT_BAD_LENGTH, "short step");
  bad = step;
  bad.h.type = 99;
  expect(checkPacket((const uint8_t *)&bad, sizeof(bad)) == PKT_BAD_LENGTH, "unknown type");
  uint8_t legacy[LEGACY_PACKET_LEN] = {1, 0, 2, 0, 0, 1};
  expect(checkPacket(legacy, sizeof(legacy)) == PKT_BAD_LENGTH, "legacy packet");

  return failures ? 1 : 0;
}
