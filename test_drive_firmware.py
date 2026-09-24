"""
Checkpoint E/F: the drive link, Pi side and firmware side (decision #77, checkpoint F).

    python3 test_drive_firmware.py

- the firmware's protocol half (firmware/drive_bridge/drive_protocol.h) is
  compiled on the host with g++ and fed the exact bytes drive_link.py sends,
  with garbage, a corrupted frame and frames split across reads mixed in;
  the PI_READY / RUN_OVER flag bits
- the steering map against config.py, the 250 ms watchdog, no motor outside RUNNING
- the start button (OpenRound's debounce) and the run-state machine: start,
  stop, restart, finish, a press while the Pi is not ready
- the speed loop against a motor model (placeholder gains: this checks the
  loop, not the tuning)
- drive_bridge.ino compiles against stub Arduino / HAL / library headers
  (catches C++ errors; the real build is the Arduino IDE's)
- stm32_link reads both $STA forms (never counted as bad) and DriveLink refuses
  motion while the link is not healthy, through a real pseudo-terminal
"""
from __future__ import annotations

import os
import pty
import re
import shutil
import subprocess
import tempfile
import time

import config
from drive_link import DriveLink, encode, FLAG_PI_READY, FLAG_RUN_OVER
from follower import DriveCmd
from stm32_link import Stm32Link

HERE = os.path.dirname(os.path.abspath(__file__))
FW = os.path.join(HERE, "firmware", "drive_bridge")
HARNESS = r'''
#include "drive_protocol.h"
#include <stdio.h>
#include <math.h>
using namespace drive;
int main() {
  Parser p; Command c; SteerMap m;
  int ch;
  while ((ch = getchar()) != EOF) {
    if (p.feed((uint8_t)ch, c))
      printf("CMD %u %d %d %d %d %d %d %d\n", c.seq, c.enable, c.closedLoop, (int)c.mode, c.steerDdeg, c.speedMmps,
             c.piReady, c.runOver);
  }
  printf("GOOD %u BAD %u\n", p.good, p.bad);
  printf("LOCK %.2f %.2f %.2f %.2f %.2f\n", m.lockLeftDeg, m.lockRightDeg, m.straight, m.leftStop, m.rightStop);
  float angles[] = {0.0f, m.lockLeftDeg, -m.lockRightDeg, m.lockLeftDeg / 2, -m.lockRightDeg / 2, 90.0f, -90.0f};
  for (float a : angles) printf("SERVO %.2f %.3f\n", a, m.servoFor(a));

  Command d; d.enable = true; d.mode = DIRECT; d.steerDdeg = 100; d.speedMmps = 500;
  Output o1 = decide(d, true, 100, true), o2 = decide(d, true, 300, true), o3 = decide(d, false, 0, true),
         o4 = decide(d, true, 100, false);
  printf("WD %d %d %d %d %d %d\n", o1.motorOn, o1.watchdog, o2.motorOn, o2.watchdog, o3.motorOn, o4.motorOn);

  // button: raw level every 10 ms from a script (1 = released / HIGH)
  {
    Button b; uint32_t t = 0; int presses = 0;
    auto run = [&](int level, int ms) { for (int i = 0; i < ms; i += 10, t += 10) presses += b.update(level, t); };
    run(0, 500);  printf("BTN held_at_boot %d\n", presses);          // held since power-up: no press
    run(1, 200);  printf("BTN released %d\n", presses);
    run(0, 10); run(1, 10); run(0, 10); run(1, 200);                  // 10 ms bounces: no press
    printf("BTN bounce %d\n", presses);
    run(0, 100); printf("BTN press1 %d held %d\n", presses, (int)b.held());
    run(1, 100); run(0, 100); printf("BTN lockout %d\n", presses);    // 2nd press 200 ms after the 1st
    run(1, 400); run(0, 100); printf("BTN press2 %d\n", presses);
  }
  // run-state machine
  {
    Run r; Command over; over.runOver = true; Command plain;
    printf("RUN");
    { int e = r.press(false); printf(" %d/%d", e, r.state); }                // not ready: ignored, READY
    { int e = r.frame(over); printf(" %d/%d", e, r.state); }                 // RUN_OVER outside a run: nothing
    { int e = r.press(true); printf(" %d/%d", e, r.state); }                 // start
    { int e = r.frame(plain); printf(" %d/%d", e, r.state); }
    { int e = r.press(false); printf(" %d/%d", e, r.state); }                // stop (ready does not matter)
    { int e = r.press(true); printf(" %d/%d", e, r.state); }                 // restart
    { int e = r.frame(over); printf(" %d/%d", e, r.state); }                 // Pi ends it: FINISHED
    { int e = r.press(false); printf(" %d/%d", e, r.state); }                // not ready: stays FINISHED
    { int e = r.press(true); printf(" %d/%d id %u\n", e, r.state, r.runId); }
  }
  // speed loop vs a motor: v' = (K (pwm - sign*off) - v) / tau, K 4 mm/s per PWM,
  // offset 20, tau 0.12 s, 10 ms period, the firmware's windowed estimator
  {
    float targets[] = {200, 500, 800, -200};
    for (float tg : targets) {
      SpeedPI pi; SpeedEstimator est; float v = 0, pos = 0, peak = 0; int pwm = 0;
      float at1 = 0;
      for (int k = 0; k < 300; k++) {
        float meas = est.update((int32_t)lroundf(pos * 1.4853f), 1.4853f, 0.01f);
        pwm = pi.step(tg, meas, 0.01f, true);
        float drivev = 0;
        if (pwm > 20) drivev = 4.0f * (pwm - 20); else if (pwm < -20) drivev = 4.0f * (pwm + 20);
        v += (drivev - v) * 0.01f / 0.12f;
        pos += v * 0.01f;
        if (fabsf(v) > peak) peak = fabsf(v);
        if (k == 199) at1 = v;
      }
      printf("PI %.0f %.1f %.1f %.1f %d\n", tg, at1, v, peak, pwm);
    }
    SpeedPI z; printf("PIZERO %d\n", z.step(0.0f, 300.0f, 0.01f, true));
  }
  char b[64]; formatStatus(b, sizeof b, 42, ST_ENABLED | ST_BUTTON | ST_IMU_OK, RUNNING, 3, -87, -201.6f);
  printf("STA %s", b);
  return 0;
}
'''

STUBS = {
    "Arduino.h": r'''
#pragma once
#include <stdint.h>
#include <math.h>
#include <stdio.h>
#define PA0 0
#define PA1 1
#define PA2 2
#define PA3 3
#define PA4 4
#define PA5 5
#define PA6 6
#define PA7 7
#define PA8 8
#define PB0 16
#define PB1 17
#define PB12 28
#define PC13 45
#define HIGH 1
#define LOW 0
#define OUTPUT 1
#define INPUT_PULLUP 2
#define PI 3.14159265f
inline void pinMode(int, int) {}
inline void digitalWrite(int, int) {}
inline int digitalRead(int) { return HIGH; }
inline void analogWrite(int, int) {}
inline uint32_t millis() { return 0; }
inline void delay(uint32_t) {}
inline uint32_t HAL_GetTick() { return 0; }
struct SerialT {
  void begin(long) {}
  explicit operator bool() const { return true; }
  int available() { return 0; }
  int read() { return -1; }
  int availableForWrite() { return 64; }
  size_t write(const uint8_t *, size_t n) { return n; }
};
extern SerialT Serial;
struct GPIO_InitTypeDef { uint32_t Pin, Mode, Pull, Speed, Alternate; };
struct TIM_Base_InitTypeDef { uint32_t Prescaler, CounterMode, Period; };
struct TIM_TypeDef { volatile uint32_t CNT; };
struct TIM_HandleTypeDef { TIM_TypeDef *Instance; TIM_Base_InitTypeDef Init; };
struct TIM_Encoder_InitTypeDef { uint32_t EncoderMode, IC1Polarity, IC1Selection, IC2Polarity, IC2Selection; };
extern TIM_TypeDef tim5_;
extern void *GPIOA;
#define TIM5 (&tim5_)
#define __HAL_RCC_GPIOA_CLK_ENABLE() do {} while (0)
#define __HAL_RCC_TIM5_CLK_ENABLE() do {} while (0)
#define GPIO_PIN_0 1
#define GPIO_PIN_1 2
#define GPIO_MODE_AF_PP 2
#define GPIO_PULLUP 1
#define GPIO_SPEED_FREQ_HIGH 2
#define GPIO_AF2_TIM5 2
#define TIM_COUNTERMODE_UP 0
#define TIM_ENCODERMODE_TI12 3
#define TIM_ICPOLARITY_RISING 0
#define TIM_ICSELECTION_DIRECTTI 1
#define TIM_CHANNEL_ALL 0x3C
inline void HAL_GPIO_Init(void *, GPIO_InitTypeDef *) {}
inline int HAL_TIM_Encoder_Init(TIM_HandleTypeDef *, TIM_Encoder_InitTypeDef *) { return 0; }
inline int HAL_TIM_Encoder_Start(TIM_HandleTypeDef *, uint32_t) { return 0; }
''',
    "Servo.h": "#pragma once\nstruct Servo { void attach(int, int, int) {} void writeMicroseconds(int) {} };\n",
    "SPI.h": "#pragma once\nstruct SPIClass { SPIClass(int, int, int) {} void begin() {} };\n",
    "SparkFun_BNO08x_Arduino_Library.h": r'''
#pragma once
#define SENSOR_REPORTID_GAME_ROTATION_VECTOR 0x08
struct BNO08x {
  bool beginSPI(int, int, int, long, SPIClass &) { return true; }
  bool wasReset() { return false; }
  bool enableGameRotationVector(uint16_t = 10) { return true; }
  bool getSensorEvent() { return false; }
  uint8_t getSensorEventID() { return 0; }
  float getQuatI() { return 0; } float getQuatJ() { return 0; } float getQuatK() { return 0; }
  float getQuatReal() { return 1; }
};
''',
}
INO_MAIN = r'''
#include "Arduino.h"
SerialT Serial; TIM_TypeDef tim5_; void *GPIOA = 0;
#include "drive_bridge.ino.cpp"
int main() { setup(); for (int i = 0; i < 10; i++) loop(); return 0; }
'''


def _gxx():
    gxx = shutil.which("g++")
    if gxx is None:
        print("SKIP  (no g++)")
    return gxx


def test_firmware_protocol():
    gxx = _gxx()
    if gxx is None:
        return
    d = tempfile.mkdtemp()
    src, exe = os.path.join(d, "h.cpp"), os.path.join(d, "h")
    open(src, "w").write(HARNESS)
    subprocess.check_call([gxx, "-std=c++17", "-Wall", "-Wextra", "-Werror", "-I", FW, src, "-o", exe])
    cmds = [DriveCmd(12.3, 500), DriveCmd(-40.5, 800), DriveCmd(0.0, 0.0, stop=True), DriveCmd(36.6, 120)]
    frames = [encode(cmds[0], 7), encode(cmds[1], 8, ready=True), encode(cmds[2], 9, ready=True, run_over=True),
              encode(cmds[3], 10, closed_loop=False)]
    assert frames[1][3] & FLAG_PI_READY and frames[2][3] & FLAG_RUN_OVER
    bad = bytearray(frames[1])
    bad[6] ^= 0x10                                         # corrupt one body byte of a copy
    stream = b"\x00\x55\xAA" + frames[0] + b"garbage" + bytes(bad) + frames[1] + frames[2][:5] + frames[2][5:] \
        + b"\xAA" + frames[3]
    out = subprocess.run([exe], input=stream, capture_output=True, check=True).stdout.decode()
    lines = out.splitlines()

    def rows(tag):
        return [ln.split()[1:] for ln in lines if ln.startswith(tag + " ")]

    got = rows("CMD")
    want = [["7", "1", "1", "0", "123", "500", "0", "0"], ["8", "1", "1", "0", "-405", "800", "1", "0"],
            ["9", "0", "0", "2", "0", "0", "1", "1"], ["10", "1", "0", "0", "366", "120", "0", "0"]]
    assert got == want, (got, want)
    gb = rows("GOOD")[0]
    assert gb[0] == "4" and int(gb[2]) >= 1, gb

    lock = [float(v) for v in rows("LOCK")[0]]
    assert lock == [config.STEER_LOCK_LEFT_DEG, config.STEER_LOCK_RIGHT_DEG, config.SERVO_STRAIGHT_DEG,
                    config.SERVO_LEFT_STOP_DEG, config.SERVO_RIGHT_STOP_DEG], ("firmware and config.py disagree", lock)
    servo = [(float(a), float(s)) for a, s in rows("SERVO")]
    st, ls, rs = config.SERVO_STRAIGHT_DEG, config.SERVO_LEFT_STOP_DEG, config.SERVO_RIGHT_STOP_DEG
    want_s = [st, ls, rs, (st + ls) / 2, (st + rs) / 2, ls, rs]
    assert all(abs(s - w) < 2e-3 for (_, s), w in zip(servo, want_s)), servo

    assert rows("WD")[0] == ["1", "0", "0", "1", "0", "0"], rows("WD")

    btn = {r[0]: r[1:] for r in rows("BTN")}
    assert btn["held_at_boot"] == ["0"] and btn["released"] == ["0"] and btn["bounce"] == ["0"], btn
    assert btn["press1"] == ["1", "held", "1"] and btn["lockout"] == ["1"] and btn["press2"] == ["2"], btn

    # EV: 0 NONE 1 START 2 STOP 3 FINISH 4 IGNORED;  state: 0 READY 1 RUNNING 2 STOPPED 3 FINISHED
    run = rows("RUN")[0]
    assert run == ["4/0", "0/0", "1/1", "0/1", "2/2", "1/1", "3/3", "4/3", "1/1", "id", "3"], run

    for tg, at1, v, peak, pwm in rows("PI"):
        tg, at1, v, peak = float(tg), float(at1), float(v), float(peak)
        assert abs(at1 - tg) < 0.05 * abs(tg) and abs(v - tg) < 0.03 * abs(tg), (tg, at1, v)
        assert peak < 1.25 * abs(tg), (tg, peak)
        assert (int(pwm) > 0) == (tg > 0), (tg, pwm)
    assert rows("PIZERO")[0] == ["0"]

    sta = [ln for ln in lines if ln.startswith("STA")][0]
    assert sta.strip() == "STA $STA,42,21,1,3,-87,-202", sta
    print("PASS  test_firmware_protocol   4/4 frames decoded from drive_link bytes (PI_READY / RUN_OVER bits) through "
          f"garbage, a corrupted frame (dropped, bad={gb[2]}) and a split frame; servo map = config.py; watchdog; "
          "no motor outside RUNNING; button debounce (held at boot, bounce, lockout); run states "
          "start/stop/restart/finish/ignored; speed loop within 5 % in 2 s, overshoot < 25 %, reverse; $STA format")


def test_sketch_compiles():
    gxx = _gxx()
    if gxx is None:
        return
    d = tempfile.mkdtemp()
    for name, text in STUBS.items():
        open(os.path.join(d, name), "w").write(text)
    shutil.copy(os.path.join(FW, "drive_bridge.ino"), os.path.join(d, "drive_bridge.ino.cpp"))
    shutil.copy(os.path.join(FW, "drive_protocol.h"), d)
    src = os.path.join(d, "main.cpp")
    open(src, "w").write(INO_MAIN)
    r = subprocess.run([gxx, "-std=gnu++17", "-Wall", "-Wextra", "-Wno-unused-parameter", "-Wno-missing-field-initializers", "-Werror", "-I", d, src,
                        "-o", os.path.join(d, "ino")], capture_output=True, text=True)
    assert r.returncode == 0, r.stderr
    subprocess.run([os.path.join(d, "ino")], check=True, timeout=10)
    # the sketch's pins are OpenRound's
    ino = open(os.path.join(FW, "drive_bridge.ino")).read()
    pins = dict(re.findall(r"const int (\w+_PIN)\s*=\s*(\w+);", ino))
    assert pins == {"MOT_RPWM_PIN": "PA2", "MOT_LPWM_PIN": "PA3", "SERVO_PIN": "PA8", "IMU_CS_PIN": "PA4",
                    "IMU_INT_PIN": "PB0", "IMU_RST_PIN": "PB1", "BTN_PIN": "PB12", "STATUS_LED_PIN": "PC13"}, pins
    print("PASS  test_sketch_compiles     drive_bridge.ino builds with -Wall -Wextra -Werror against stub Arduino/HAL/"
          "library headers and runs setup()+loop(); pins are OpenRound.cpp's")


def test_link_status_and_refusal():
    master, slave = pty.openpty()
    os.set_blocking(master, False)
    import tty
    tty.setraw(slave)
    link = Stm32Link(stream=os.fdopen(slave, "r+b", buffering=0))
    link.start()
    drv = DriveLink(link)
    time.sleep(0.05)
    # no $STA yet -> motion refused (a STOP frame goes out instead)
    drv.send(DriveCmd(10.0, 400))
    for k in range(30):
        os.write(master, f"$IMU,{k + 1},{k * 10},{k * 3},1.00\n".encode())
        if k % 5 == 0:
            os.write(master, f"$STA,{k % 256},{1 | 4 | 16},1,2,-40,-150\n".encode())
        time.sleep(0.01)
    time.sleep(0.05)
    st = link.status()
    assert st["lines_bad"] == 0 and st["sta"] is not None and st["sta"]["status"] == 21, st
    assert st["sta"]["run_state"] == 1 and st["sta"]["run_id"] == 2 and st["sta"]["pwm"] == -40 \
        and st["sta"]["speed_mm_s"] == -150, st
    assert drv.healthy() and drv.button_held() and drv.run_state() == (1, 2)
    drv.ready = True
    drv.send(DriveCmd(10.0, 400))
    time.sleep(0.05)
    data = b""
    try:
        data = os.read(master, 4096)
    except BlockingIOError:
        pass
    assert data[:11] == encode(DriveCmd(0, 0, stop=True), 0) and \
        data[11:22] == encode(DriveCmd(10.0, 400), 1, ready=True), data.hex()
    # the short (checkpoint-E) form still parses; no run state from it
    os.write(master, b"$STA,1,21\n")
    time.sleep(0.05)
    assert link.status()["lines_bad"] == 0 and drv.run_state() == (None, None)
    # watchdog bit -> not healthy
    os.write(master, b"$STA,1,3,1,2,0,0\n$IMU,99,990,90,1.00\n")
    time.sleep(0.05)
    assert not drv.healthy()
    link.stop()
    print("PASS  test_link_status_and_refusal both $STA forms parsed (0 bad lines), run state / id / pwm / speed read; "
          "motion before any $STA and with the WATCHDOG bit is refused (STOP sent); PI_READY reaches the port "
          "byte-exact")


if __name__ == "__main__":
    test_firmware_protocol()
    test_sketch_compiles()
    test_link_status_and_refusal()
    print("\nAll drive-link checks passed.")
