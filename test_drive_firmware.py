"""
Checkpoint E: the drive link, Pi side and firmware side (decision #77).

    python3 test_drive_firmware.py

- the firmware's protocol half (firmware/drive_bridge/drive_protocol.h) is
  compiled on the host with g++ and fed the exact bytes drive_link.py sends,
  with garbage, a corrupted frame and frames split across reads mixed in
- the steering map and the 250 ms watchdog
- stm32_link reads $STA lines (never counted as bad) and DriveLink refuses
  motion while the link is not healthy, through a real pseudo-terminal
"""
from __future__ import annotations

import os
import pty
import shutil
import subprocess
import tempfile
import time

import config
from drive_link import DriveLink, encode
from follower import DriveCmd
from stm32_link import Stm32Link

HERE = os.path.dirname(os.path.abspath(__file__))
HARNESS = r'''
#include "drive_protocol.h"
#include <stdio.h>
int main() {
  drive::Parser p; drive::Command c; drive::SteerMap m;
  int ch;
  while ((ch = getchar()) != EOF) {
    if (p.feed((uint8_t)ch, c))
      printf("CMD %u %d %d %d %d %d\n", c.seq, c.enable, c.closedLoop, (int)c.mode, c.steerDdeg, c.speedMmps);
  }
  printf("GOOD %u BAD %u\n", p.good, p.bad);
  float angles[] = {0.0f, 46.8f, -54.6f, 23.4f, -27.3f, 90.0f, -90.0f};
  for (float a : angles) printf("SERVO %.1f %.2f\n", a, m.servoFor(a));
  drive::Command d; d.enable = true; d.mode = drive::DIRECT; d.steerDdeg = 100; d.speedMmps = 500;
  drive::Output o1 = drive::decide(d, true, 100), o2 = drive::decide(d, true, 300), o3 = drive::decide(d, false, 0);
  printf("WD %d %d %d %d %d\n", o1.motorOn, o1.watchdog, o2.motorOn, o2.watchdog, o3.motorOn);
  char b[32]; drive::formatStatus(b, sizeof b, 42, drive::ST_ENABLED | drive::ST_BUTTON | drive::ST_IMU_OK);
  printf("STA %s", b);
  return 0;
}
'''


def test_firmware_parser():
    gxx = shutil.which("g++")
    if gxx is None:
        print("SKIP  test_firmware_parser (no g++)")
        return
    d = tempfile.mkdtemp()
    src = os.path.join(d, "h.cpp")
    exe = os.path.join(d, "h")
    open(src, "w").write(HARNESS)
    subprocess.check_call([gxx, "-std=c++17", "-Wall", "-Wextra", "-Werror", "-I",
                           os.path.join(HERE, "firmware", "drive_bridge"), src, "-o", exe])
    cmds = [DriveCmd(12.3, 500), DriveCmd(-54.6, 800), DriveCmd(0.0, 0.0, stop=True), DriveCmd(46.8, 120)]
    frames = [encode(c, i + 7) for i, c in enumerate(cmds)]
    bad = bytearray(frames[1])
    bad[6] ^= 0x10                                         # corrupt one body byte of a copy
    stream = b"\x00\x55\xAA" + frames[0] + b"garbage" + bytes(bad) + frames[1] + frames[2][:5] + frames[2][5:] \
        + b"\xAA" + frames[3]
    out = subprocess.run([exe], input=stream, capture_output=True, check=True).stdout.decode()
    got = [ln.split()[1:] for ln in out.splitlines() if ln.startswith("CMD")]
    want = [["7", "1", "1", "0", "123", "500"], ["8", "1", "1", "0", "-546", "800"],
            ["9", "0", "0", "2", "0", "0"], ["10", "1", "1", "0", "468", "120"]]
    assert got == want, (got, want)
    gb = [ln for ln in out.splitlines() if ln.startswith("GOOD")][0].split()
    assert gb[1] == "4" and int(gb[3]) >= 1, gb
    servo = {float(ln.split()[1]): float(ln.split()[2]) for ln in out.splitlines() if ln.startswith("SERVO")}
    assert abs(servo[0.0] - config.SERVO_STRAIGHT_DEG) < 1e-6
    assert abs(servo[46.8] - config.SERVO_LEFT_STOP_DEG) < 1e-3 and abs(servo[-54.6] - config.SERVO_RIGHT_STOP_DEG) < 1e-3
    assert servo[90.0] == config.SERVO_LEFT_STOP_DEG and servo[-90.0] == config.SERVO_RIGHT_STOP_DEG
    wd = [ln for ln in out.splitlines() if ln.startswith("WD")][0].split()[1:]
    assert wd == ["1", "0", "0", "1", "0"], wd
    sta = [ln for ln in out.splitlines() if ln.startswith("STA")][0]
    assert sta.strip() == "STA $STA,42,21", sta
    print("PASS  test_firmware_parser     4/4 frames decoded from drive_link bytes through garbage, a corrupted frame "
          f"(dropped, bad={gb[3]}) and a split frame; servo map straight/stops/clamp; watchdog at 250 ms; $STA format")


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
            os.write(master, f"$STA,{k % 256},{1 | 4 | 16}\n".encode())
        time.sleep(0.01)
    time.sleep(0.05)
    st = link.status()
    assert st["lines_bad"] == 0 and st["sta"] is not None and st["sta"]["status"] == 21, st
    assert drv.healthy() and drv.button_pressed()
    drv.send(DriveCmd(10.0, 400))
    time.sleep(0.05)
    data = b""
    try:
        data = os.read(master, 4096)
    except BlockingIOError:
        pass
    assert data[:11] == encode(DriveCmd(0, 0, stop=True), 0) and data[11:22] == encode(DriveCmd(10.0, 400), 1), data.hex()
    # watchdog bit -> not healthy
    os.write(master, b"$STA,1,3\n$IMU,99,990,90,1.00\n")
    time.sleep(0.05)
    assert not drv.healthy()
    link.stop()
    print("PASS  test_link_status_and_refusal $STA parsed (0 bad lines), button bit seen; motion before any $STA and "
          "with the WATCHDOG bit is refused (STOP sent); frames reach the port byte-exact")


if __name__ == "__main__":
    test_firmware_parser()
    test_link_status_and_refusal()
    print("\nAll drive-link checks passed.")
