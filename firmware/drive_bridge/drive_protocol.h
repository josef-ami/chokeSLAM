// ============================================================
// drive_protocol.h -- the hardware-free half of the drive bridge
// (checkpoint E #77, revised in checkpoint F). Plain C++: host-testable with
// g++ (test_drive_firmware.py builds it against frames made by drive_link.py,
// and runs the run-state machine, the button debounce and the speed loop
// against scripted inputs).
//
// DRIVE frame (Pi -> STM32), the owner's stm_link.py spec, 11 bytes:
//   0xAA 0x55 | seq u8 | flags u8 | steer_ddeg i16 | speed_mmps i16 | heading_ddeg i16 | xor8
//   flags: bit0 ENABLE, bit1 CLOSED_LOOP, bits 2-3 mode (0 DIRECT, 1 HEADING_HOLD, 2 STOP)
//          chokeSLAM additions (bits the owner's spec leaves unused):
//          bit4 PI_READY  the Pi holds a valid initialisation: a start press may start a run
//          bit5 RUN_OVER  the Pi's run has ended (finished or failed): RUNNING -> FINISHED
//   steer_ddeg: road-wheel angle x10, + = LEFT; little-endian; xor8 over the 8 body bytes.
// STATUS line (STM32 -> Pi), ASCII, 20 Hz:
//   $STA,<seq_ack>,<status>,<run_state>,<run_id>,<pwm>,<speed_mmps>\n
//   status bits: 0 ENABLED (motor under DRIVE control now), 1 WATCHDOG, 2 BUTTON
//                (held down now, debounced), 3 CLOSED_LOOP, 4 IMU_OK
//   run_state: 0 READY, 1 RUNNING, 2 STOPPED, 3 FINISHED;  run_id: runs started since boot
//   pwm: motor PWM applied (-255..255);  speed_mmps: the encoder speed estimate
//
// PARAM frame (Pi -> STM32, checkpoint F2), 8 bytes: the tuning values below
// can be changed live, with no re-flash:
//   0xAA 0x56 | id u8 | value float32 little-endian | xor8 over id and the 4 value bytes
//   id 0xFF (value ignored) = report every value. Every accepted or reported value
//   is echoed as  $PAR,<id>,<value>\n ; a value outside its range is clamped
//   (the echo shows what was applied). Values fall back to the compiled
//   defaults at power-up; the Pi re-sends config.py's (drive_link.sync_params).
//
// RUN STATE (the owner's button rule, checkpoint F):
//   READY / STOPPED / FINISHED --press, Pi ready--> RUNNING (run_id + 1)
//   RUNNING --press--> STOPPED          (motor off at once, whatever the Pi sends)
//   RUNNING --RUN_OVER frame--> FINISHED
//   A press while the Pi is not ready is ignored (the status LED says so).
//   The motor runs only in RUNNING, and only on fresh DIRECT frames.
// ============================================================
#pragma once
#include <stdint.h>
#include <stdio.h>
#include <string.h>

namespace drive {

enum Mode : uint8_t { DIRECT = 0, HEADING_HOLD = 1, STOP = 2 };
enum RunState : uint8_t { READY = 0, RUNNING = 1, STOPPED = 2, FINISHED = 3 };

const uint8_t SYNC0 = 0xAA, SYNC1 = 0x55, SYNC1_PARAM = 0x56;
const uint8_t FRAME_LEN = 11, PARAM_LEN = 8;
const uint32_t WATCHDOG_MS = 250;

const uint8_t FL_ENABLE = 1 << 0, FL_CLOSED_LOOP = 1 << 1, FL_PI_READY = 1 << 4, FL_RUN_OVER = 1 << 5;
const uint8_t ST_ENABLED = 1 << 0, ST_WATCHDOG = 1 << 1, ST_BUTTON = 1 << 2, ST_CLOSED_LOOP = 1 << 3,
              ST_IMU_OK = 1 << 4;

struct Command {
  uint8_t seq = 0;
  bool enable = false;
  bool closedLoop = false;
  bool piReady = false;
  bool runOver = false;
  Mode mode = STOP;
  int16_t steerDdeg = 0;
  int16_t speedMmps = 0;
  int16_t headingDdeg = 0;
};

struct ParamMsg {
  uint8_t id = 0;
  float value = 0.0f;
};

// Byte-at-a-time parser for both frames: resyncs by sliding one byte on any
// mismatch; a frame failing its checksum is dropped and counted, never acted on.
class Parser {
 public:
  uint32_t good = 0, bad = 0, params = 0;
  enum Kind : uint8_t { NONE = 0, DRIVE = 1, PARAM = 2 };
  // returns DRIVE when `out` holds a new valid command, PARAM when `pm` holds a parameter
  Kind feed(uint8_t b, Command &out, ParamMsg &pm) {
    buf_[n_++] = b;
    while (n_ > 0) {
      if (buf_[0] != SYNC0 || (n_ > 1 && buf_[1] != SYNC1 && buf_[1] != SYNC1_PARAM)) { shift(1); continue; }
      if (n_ < 2) return NONE;
      uint8_t len = buf_[1] == SYNC1 ? FRAME_LEN : PARAM_LEN;
      if (n_ < len) return NONE;
      uint8_t x = 0;
      for (int i = 2; i < len - 1; i++) x ^= buf_[i];
      if (x != buf_[len - 1]) { bad++; shift(1); continue; }
      if (len == PARAM_LEN) {
        pm.id = buf_[2];
        uint32_t u = (uint32_t)buf_[3] | ((uint32_t)buf_[4] << 8) | ((uint32_t)buf_[5] << 16) | ((uint32_t)buf_[6] << 24);
        memcpy(&pm.value, &u, 4);
        shift(PARAM_LEN);
        params++;
        return PARAM;
      }
      out.seq = buf_[2];
      uint8_t f = buf_[3];
      out.enable = f & FL_ENABLE;
      out.closedLoop = f & FL_CLOSED_LOOP;
      out.piReady = f & FL_PI_READY;
      out.runOver = f & FL_RUN_OVER;
      uint8_t m = (f >> 2) & 3;
      out.mode = m == 0 ? DIRECT : (m == 1 ? HEADING_HOLD : STOP);
      out.steerDdeg = (int16_t)(buf_[4] | (buf_[5] << 8));
      out.speedMmps = (int16_t)(buf_[6] | (buf_[7] << 8));
      out.headingDdeg = (int16_t)(buf_[8] | (buf_[9] << 8));
      shift(FRAME_LEN);
      good++;
      return DRIVE;
    }
    return NONE;
  }
  // the checkpoint-E signature: DRIVE frames only
  bool feed(uint8_t b, Command &out) {
    ParamMsg pm;
    return feed(b, out, pm) == DRIVE;
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

// The start button, OpenRound's non-blocking debounce: a level must hold
// DEBOUNCE_MS to count, a press closer than LOCKOUT_MS to the last one is
// ignored, and `stable` starts LOW ("pressed") so the first edge that can be
// seen is a RELEASE: a button held (or shorted) at power-up never starts a run.
struct Button {
  static const uint32_t DEBOUNCE_MS = 30, LOCKOUT_MS = 400;
  bool lastRaw = false, stable = false;      // false = LOW = pressed (pull-up to GND)
  uint32_t changeMs = 0, lastPressMs = 0;
  bool pressedOnce = false;
  // raw: the pin level (true = HIGH = released). Returns true once per press.
  bool update(bool raw, uint32_t nowMs) {
    if (raw != lastRaw) { lastRaw = raw; changeMs = nowMs; }
    if (raw != stable && nowMs - changeMs >= DEBOUNCE_MS) {
      stable = raw;
      if (!stable) {
        if (pressedOnce && nowMs - lastPressMs < LOCKOUT_MS) return false;
        pressedOnce = true;
        lastPressMs = nowMs;
        return true;
      }
    }
    return false;
  }
  bool held() const { return !stable; }
};

// The run-state machine above. Events are returned for the '#' log line.
enum RunEvent : uint8_t { EV_NONE = 0, EV_START, EV_STOP, EV_FINISH, EV_IGNORED };

struct Run {
  RunState state = READY;
  uint32_t runId = 0;
  // a debounced press; piReady = a fresh frame (<= WATCHDOG_MS) carrying PI_READY, IMU healthy
  RunEvent press(bool piReady) {
    if (state == RUNNING) { state = STOPPED; return EV_STOP; }
    if (!piReady) return EV_IGNORED;
    state = RUNNING;
    runId++;
    return EV_START;
  }
  // every valid frame
  RunEvent frame(const Command &c) {
    if (state == RUNNING && c.runOver) { state = FINISHED; return EV_FINISH; }
    return EV_NONE;
  }
};

// Road-wheel angle (deg, + = LEFT) -> servo angle (deg). Linear between
// straight and each stop (config.py SERVO_*, STEER_LOCK_*): below straight
// steers LEFT. Clamped to the stops.
// lockLeftDeg / lockRightDeg: the bicycle-equivalent wheel angle at each stop.
// PLACEHOLDERS (owner, checkpoint F): converted from the owner's full-lock radii
// (270 mm left, 250 mm right, computed from encoder distance / IMU heading
// change) as if taken at the outer front wheel; see CHANGES 17.2. Live-settable
// (PARAM ids 4, 5); config.py STEER_LOCK_* is the source of truth.
struct SteerMap {
  float straight = 76.5f, leftStop = 20.0f, rightStop = 140.0f;
  float lockLeftDeg = 36.6f, lockRightDeg = 40.5f;
  float servoFor(float wheelDeg) const {
    float s;
    if (wheelDeg >= 0) s = straight - wheelDeg / lockLeftDeg * (straight - leftStop);
    else               s = straight + (-wheelDeg) / lockRightDeg * (rightStop - straight);
    if (s < leftStop) s = leftStop;
    if (s > rightStop) s = rightStop;
    return s;
  }
};

// What the actuators should do now, given the run state, the last valid
// command and its age.
struct Output {
  bool motorOn;
  float wheelDeg;      // road-wheel angle, + = LEFT
  float speedMmps;
  bool closedLoop;
  bool watchdog;
};

inline Output decide(const Command &c, bool haveCmd, uint32_t ageMs, bool running) {
  Output o{false, 0.0f, 0.0f, false, false};
  if (!haveCmd || ageMs > WATCHDOG_MS) { o.watchdog = haveCmd; return o; }
  // HEADING_HOLD is not used by the Pi (it steers with DIRECT from its own
  // pose); treated as STOP here rather than half-implemented.
  if (!running || !c.enable || c.mode != DIRECT) return o;
  o.motorOn = true;
  o.wheelDeg = c.steerDdeg / 10.0f;
  o.speedMmps = c.speedMmps;
  o.closedLoop = c.closedLoop;
  return o;
}

// Encoder speed over the last N control periods (a window, not a single
// period: at 10 ms and 1.49 ticks/mm one tick is 67 mm/s).
struct SpeedEstimator {
  static const int N = 5;
  int32_t ring[N + 1] = {0};
  int k = 0, filled = 0;
  float update(int32_t enc, float ticksPerMm, float periodS) {
    ring[k] = enc;
    int oldest = (k + 1) % (N + 1);
    k = oldest;
    if (filled < N) { filled++; return 0.0f; }
    return (enc - ring[oldest]) / ticksPerMm / (N * periodS);
  }
};

// Speed loop (PWM counts): feed-forward kff * v + offset (the PWM the motor
// needs to start turning), plus PI on the encoder speed when CLOSED_LOOP.
// The integrator is reset on a zero target or a direction change and frozen
// while the output is saturated. Values are PLACEHOLDERS until
// drive_calibrate.py has measured the motor (docs/RUNNING_ON_THE_ROBOT.md).
// Live-settable (PARAM ids 6-9); config.py SPEED_* is the source of truth.
struct SpeedPI {
  float kff = 0.20f;      // PWM per mm/s
  float offset = 25.0f;   // PWM
  float kp = 0.10f;       // PWM per mm/s of error
  float ki = 0.50f;       // PWM per mm of accumulated error
  float integ = 0.0f;     // mm
  float lastTarget = 0.0f;
  int pwmMax = 255;
  int step(float target, float measured, float dt, bool closedLoop) {
    if (target == 0.0f || (target > 0) != (lastTarget > 0)) integ = 0.0f;
    lastTarget = target;
    if (target == 0.0f) return 0;
    float sgn = target > 0 ? 1.0f : -1.0f;
    float u = kff * target + sgn * offset;
    if (closedLoop) {
      float e = target - measured;
      float trial = u + kp * e + ki * (integ + e * dt);
      bool sat = trial > pwmMax || trial < -pwmMax;
      if (!sat || (trial > 0) != (e > 0)) integ += e * dt;   // no wind-up into the limit
      u += kp * e + ki * integ;
    }
    // never drive against the requested direction (it would read as a reversal)
    if (u * sgn < 0) u = 0;
    int p = (int)(u + (u >= 0 ? 0.5f : -0.5f));
    if (p > pwmMax) p = pwmMax;
    if (p < -pwmMax) p = -pwmMax;
    return p;
  }
};

// The live-tunable values (PARAM frame). Ids are fixed: drive_link.FW_PARAMS mirrors them.
enum ParamId : uint8_t {
  P_SERVO_STRAIGHT = 1, P_SERVO_LEFT_STOP = 2, P_SERVO_RIGHT_STOP = 3, P_LOCK_LEFT = 4, P_LOCK_RIGHT = 5,
  P_KFF = 6, P_OFFSET = 7, P_KP = 8, P_KI = 9, P_TICKS_PER_MM = 10, P_COUNT = 11, P_REPORT_ALL = 0xFF
};

struct Tunables {
  SteerMap *steer;
  SpeedPI *pi;
  float *ticksPerMm;
  static float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
  // apply a value (clamped to its range); false for an unknown id or a non-finite value
  bool set(uint8_t id, float v) {
    if (!(v == v) || v > 1e9f || v < -1e9f) return false;
    switch (id) {
      case P_SERVO_STRAIGHT:   steer->straight = clampf(v, steer->leftStop + 1.0f, steer->rightStop - 1.0f); break;
      case P_SERVO_LEFT_STOP:  steer->leftStop = clampf(v, 0.0f, steer->straight - 1.0f); break;
      case P_SERVO_RIGHT_STOP: steer->rightStop = clampf(v, steer->straight + 1.0f, 180.0f); break;
      case P_LOCK_LEFT:        steer->lockLeftDeg = clampf(v, 5.0f, 80.0f); break;
      case P_LOCK_RIGHT:       steer->lockRightDeg = clampf(v, 5.0f, 80.0f); break;
      case P_KFF:              pi->kff = clampf(v, 0.0f, 5.0f); break;
      case P_OFFSET:           pi->offset = clampf(v, 0.0f, 200.0f); break;
      case P_KP:               pi->kp = clampf(v, 0.0f, 5.0f); break;
      case P_KI:               pi->ki = clampf(v, 0.0f, 20.0f); pi->integ = 0.0f; break;
      case P_TICKS_PER_MM:     *ticksPerMm = clampf(v, 0.1f, 20.0f); break;
      default: return false;
    }
    return true;
  }
  bool get(uint8_t id, float &v) const {
    switch (id) {
      case P_SERVO_STRAIGHT:   v = steer->straight; break;
      case P_SERVO_LEFT_STOP:  v = steer->leftStop; break;
      case P_SERVO_RIGHT_STOP: v = steer->rightStop; break;
      case P_LOCK_LEFT:        v = steer->lockLeftDeg; break;
      case P_LOCK_RIGHT:       v = steer->lockRightDeg; break;
      case P_KFF:              v = pi->kff; break;
      case P_OFFSET:           v = pi->offset; break;
      case P_KP:               v = pi->kp; break;
      case P_KI:               v = pi->ki; break;
      case P_TICKS_PER_MM:     v = *ticksPerMm; break;
      default: return false;
    }
    return true;
  }
};

inline int formatParam(char *buf, int n, uint8_t id, float v) {
  // no %f in newlib-nano's default printf: fixed point with 4 decimals
  long scaled = (long)(v * 10000.0f + (v >= 0 ? 0.5f : -0.5f));
  unsigned long a = (unsigned long)(scaled < 0 ? -scaled : scaled);
  return snprintf(buf, n, "$PAR,%u,%s%lu.%04lu\n", (unsigned)id, scaled < 0 ? "-" : "", a / 10000UL, a % 10000UL);
}

inline int formatStatus(char *buf, int n, uint8_t seqAck, uint8_t status, RunState rs, uint32_t runId,
                        int pwm, float speedMmps) {
  int v = (int)(speedMmps + (speedMmps >= 0 ? 0.5f : -0.5f));
  return snprintf(buf, n, "$STA,%u,%u,%u,%lu,%d,%d\n", (unsigned)seqAck, (unsigned)status, (unsigned)rs,
                  (unsigned long)runId, pwm, v);
}

}  // namespace drive
