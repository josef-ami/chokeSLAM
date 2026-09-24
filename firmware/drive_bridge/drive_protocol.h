// ============================================================
// drive_protocol.h -- the hardware-free half of the drive bridge
// (checkpoint E, decision #77). Plain C++: host-testable with g++
// (test_drive_firmware.py builds it against frames made by drive_link.py).
//
// DRIVE frame (Pi -> STM32), the owner's stm_link.py spec, 11 bytes:
//   0xAA 0x55 | seq u8 | flags u8 | steer_ddeg i16 | speed_mmps i16 | heading_ddeg i16 | xor8
//   flags: bit0 ENABLE, bit1 CLOSED_LOOP, bits 2-3 mode (0 DIRECT, 1 HEADING_HOLD, 2 STOP)
//   steer_ddeg: road-wheel angle x10, + = LEFT; little-endian; xor8 over the 8 body bytes.
// STATUS line (STM32 -> Pi), ASCII, ~20 Hz:  $STA,<seq_ack>,<status>\n
//   status bits: 0 ENABLED, 1 WATCHDOG, 2 BUTTON (latched since boot), 4 IMU_OK
// ============================================================
#pragma once
#include <stdint.h>
#include <stdio.h>
#include <string.h>

namespace drive {

enum Mode : uint8_t { DIRECT = 0, HEADING_HOLD = 1, STOP = 2 };

const uint8_t SYNC0 = 0xAA, SYNC1 = 0x55;
const uint8_t FRAME_LEN = 11;
const uint32_t WATCHDOG_MS = 250;

const uint8_t ST_ENABLED = 1 << 0, ST_WATCHDOG = 1 << 1, ST_BUTTON = 1 << 2, ST_IMU_OK = 1 << 4;

struct Command {
  uint8_t seq = 0;
  bool enable = false;
  bool closedLoop = false;
  Mode mode = STOP;
  int16_t steerDdeg = 0;
  int16_t speedMmps = 0;
  int16_t headingDdeg = 0;
};

// Byte-at-a-time parser: resyncs by sliding one byte on any mismatch; a frame
// failing its checksum is dropped and counted, never acted on.
class Parser {
 public:
  uint32_t good = 0, bad = 0;
  // returns true when `out` holds a new valid command
  bool feed(uint8_t b, Command &out) {
    buf_[n_++] = b;
    while (n_ > 0) {
      if (buf_[0] != SYNC0 || (n_ > 1 && buf_[1] != SYNC1)) { shift(1); continue; }
      if (n_ < FRAME_LEN) return false;
      uint8_t x = 0;
      for (int i = 2; i < 10; i++) x ^= buf_[i];
      if (x != buf_[10]) { bad++; shift(1); continue; }
      out.seq = buf_[2];
      uint8_t f = buf_[3];
      out.enable = f & 1;
      out.closedLoop = (f >> 1) & 1;
      uint8_t m = (f >> 2) & 3;
      out.mode = m == 0 ? DIRECT : (m == 1 ? HEADING_HOLD : STOP);
      out.steerDdeg = (int16_t)(buf_[4] | (buf_[5] << 8));
      out.speedMmps = (int16_t)(buf_[6] | (buf_[7] << 8));
      out.headingDdeg = (int16_t)(buf_[8] | (buf_[9] << 8));
      shift(FRAME_LEN);
      good++;
      return true;
    }
    return false;
  }

 private:
  uint8_t buf_[FRAME_LEN] = {0};
  uint8_t n_ = 0;
  void shift(uint8_t k) {
    if (k >= n_) { n_ = 0; return; }
    memmove(buf_, buf_ + k, n_ - k);
    n_ -= k;
  }
};

// Road-wheel angle (deg, + = LEFT) -> servo angle (deg). PLACEHOLDER linear map
// between straight and each stop (config.py SERVO_*, STEER_LOCK_*): below
// straight steers LEFT. Clamped to the stops.
struct SteerMap {
  float straight = 76.5f, leftStop = 20.0f, rightStop = 140.0f;
  float lockLeftDeg = 46.8f, lockRightDeg = 54.6f;
  float servoFor(float wheelDeg) const {
    float s;
    if (wheelDeg >= 0) s = straight - wheelDeg / lockLeftDeg * (straight - leftStop);
    else               s = straight + (-wheelDeg) / lockRightDeg * (rightStop - straight);
    if (s < leftStop) s = leftStop;
    if (s > rightStop) s = rightStop;
    return s;
  }
};

// What the actuators should do now, given the last valid command and its age.
struct Output {
  bool motorOn;
  float wheelDeg;      // road-wheel angle, + = LEFT
  float speedMmps;
  bool closedLoop;
  bool watchdog;
};

inline Output decide(const Command &c, bool haveCmd, uint32_t ageMs) {
  Output o{false, 0.0f, 0.0f, false, false};
  if (!haveCmd || ageMs > WATCHDOG_MS) { o.watchdog = haveCmd; return o; }
  // HEADING_HOLD is not used by the Pi (it steers with DIRECT from its own
  // pose); treated as STOP here rather than half-implemented.
  if (!c.enable || c.mode != DIRECT) return o;
  o.motorOn = true;
  o.wheelDeg = c.steerDdeg / 10.0f;
  o.speedMmps = c.speedMmps;
  o.closedLoop = c.closedLoop;
  return o;
}

// Speed PI with feed-forward (PWM counts). Gains are PLACEHOLDERS to tune on the robot.
struct SpeedPI {
  float kff = 0.09f;    // PWM per mm/s
  float kp = 0.08f;     // PWM per mm/s of error
  float ki = 0.25f;     // PWM per mm of accumulated error
  float integ = 0.0f;
  int pwmMax = 255;
  int step(float target, float measured, float dt, bool closedLoop) {
    if (target == 0.0f) { integ = 0.0f; return 0; }
    float u = kff * target;
    if (closedLoop) {
      float e = target - measured;
      integ += e * dt;
      float lim = pwmMax / (ki > 0 ? ki : 1.0f);
      if (integ > lim) integ = lim;
      if (integ < -lim) integ = -lim;
      u += kp * e + ki * integ;
    }
    int p = (int)(u + (u >= 0 ? 0.5f : -0.5f));
    if (p > pwmMax) p = pwmMax;
    if (p < -pwmMax) p = -pwmMax;
    return p;
  }
};

inline int formatStatus(char *buf, int n, uint8_t seqAck, uint8_t status) {
  return snprintf(buf, n, "$STA,%u,%u\n", (unsigned)seqAck, (unsigned)status);
}

}  // namespace drive
