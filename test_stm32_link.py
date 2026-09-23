"""
Verification for stm32_link.py: the $IMU line parser, the byte -> line
assembler, and the background reader end to end over a real pseudo-terminal
(bytes written in arbitrary chunks, split mid-line, with garbage and a lost
line), plus the raw-line log and its replay.

The serial port open itself (pyserial) isn't exercised: pyserial couldn't be
installed where this was written. Everything after the open is.

Run:  python3 test_stm32_link.py
"""
from __future__ import annotations

import os
import tempfile
import time

import config
import stm32_link as sl


def test_parse_line():
    s, why = sl.parse_line("$IMU,1042,10420,15873,-12.37\r\n")
    assert s == sl.ImuSample(1042, 10420, 15873, -12.37) and why == "", (s, why)
    assert sl.parse_line("$IMU,1,2,-300,179.99")[0].enc == -300            # reversing
    bad = {
        "": "empty line",
        "IMU,1,2,3,4": "doesn't start",
        "$IMU,1,2,3": "fields",
        "$IMU,1,2,3,4,5": "fields",
        "$IMU,1,2,x,4": "non-numeric",
        "$IMU,1,2,3,nan": "finite",
        "$IMU,-1,2,3,4": "negative",
        "$GPS,1,2,3,4": "doesn't start",
    }
    for line, expect in bad.items():
        s, why = sl.parse_line(line)
        assert s is None and expect in why, (line, why)
    print(f"PASS  test_parse_line               (1 valid form, {len(bad)} malformed forms rejected with a reason)")


def test_line_assembler():
    a = sl.LineAssembler()
    out = a.feed(b"$IMU,1,10,5,0.5\n$IMU,2,2") + a.feed(b"0,6,0.6\n$IM") + a.feed(b"U,3,30,7,0.7\n")
    assert [sl.parse_line(x)[0].seq for x in out] == [1, 2, 3], out
    assert a.feed(b"x" * 5000) == ["<overflow: no newline in 4 kB>"]
    print("PASS  test_line_assembler           (lines split across reads are rejoined; runaway input is capped)")


def test_link_over_pty():
    master, slave = os.openpty()
    slave_path = os.ttyname(slave)
    log = tempfile.NamedTemporaryFile(suffix=".log", delete=False).name
    os.unlink(log)
    stream = open(slave_path, "rb", buffering=0)
    link = sl.Stm32Link(port=slave_path, stream=stream)
    link.start(log_path=log)
    lines = [f"$IMU,{i},{i * 10},{i * 15},{(i * 7.5) % 360 - 180:.2f}\n" for i in range(1, 101)]
    del lines[49]                                          # seq 50 lost on the wire
    payload = ("U,0,0,0,0.00\n"                            # partial first line after connecting
               + "".join(lines[:30]) + "garbage,without,prefix\n" + "".join(lines[30:])).encode()
    for k in range(0, len(payload), 37):                    # odd chunk size: splits lines mid-way
        os.write(master, payload[k:k + 37])
        time.sleep(0.001)
    deadline = time.time() + 3.0
    got = []
    while time.time() < deadline and len(got) < 99:
        got += link.drain()
        time.sleep(0.01)
    st = link.status()
    link.stop()
    stream.close()
    os.close(master)
    os.close(slave)
    seqs = [s.seq for s in got]
    assert seqs == [i for i in range(1, 101) if i != 50], seqs[:10]
    assert st["lines_ok"] == 99 and st["lines_bad"] == 1 and st["seq_gaps"] == 1, st
    assert "doesn't start" in st["last_bad_reason"], st
    replay = sl.read_log(log)
    assert [s.seq for s in replay] == seqs, len(replay)
    os.unlink(log)
    print(f"PASS  test_link_over_pty            99 samples in order through a pty in 37-byte chunks; "
          f"partial first line ignored, 1 garbage line counted, 1 lost line counted as a seq gap; "
          f"log replays the same {len(replay)} samples")


def test_link_open_failure_is_reported():
    link = sl.Stm32Link(port="/dev/definitely-not-a-port")
    link.start()
    time.sleep(0.2)
    st = link.status()
    assert not st["thread_alive"] and st["error"] and "could not open" in st["error"], st
    print(f"PASS  test_link_open_failure        reported, not silent: {st['error'][:70]}...")


def test_firmware_log_lines_are_skipped():
    """#52: the obstacle-round firmware prints '#' log lines and '!' tuning
    replies on the same port (CRLF, from println). They are counted as log,
    kept for display, and never counted as bad or parsed as samples."""
    import tty
    master, slave = os.openpty()
    tty.setraw(slave)            # like pyserial: no CR -> LF translation, so CRLF arrives as sent
    stream = open(os.ttyname(slave), "rb", buffering=0)
    link = sl.Stm32Link(port=os.ttyname(slave), stream=stream)
    link.start()
    fw = ["# colour CH4 READY", "# zeroing yaw", "# zero yaw -12.34", "!V 6 97 1515847680",
          "!P 0 TICKS_PER_CM 0 14.8530 1.0000 100.0000 0", "# ERROR IMU not found"]
    out = fw[0] + "\r\n"                                   # a log line first after connecting
    for i in range(1, 51):
        out += f"$IMU,{i},{i * 10},{i * 3},{-0.5 * i:.2f}\n"
        if i % 10 == 0:
            out += fw[i // 10] + "\r\n"
    out += "garbage\n"
    payload = out.encode()
    for k in range(0, len(payload), 41):
        os.write(master, payload[k:k + 41])
        time.sleep(0.001)
    deadline, got = time.time() + 3.0, []
    while time.time() < deadline and len(got) < 50:
        got += link.drain()
        time.sleep(0.01)
    time.sleep(0.05)
    st = link.status()
    link.stop(); stream.close(); os.close(master); os.close(slave)
    assert [s.seq for s in got] == list(range(1, 51)), [s.seq for s in got][:5]
    assert st["lines_ok"] == 50 and st["lines_log"] == 6 and st["lines_bad"] == 1, st
    assert st["recent_log"] == fw, st["recent_log"]
    assert sl.is_log_line("  # x") and sl.is_log_line("!p 3 1.0") and not sl.is_log_line("$IMU,1,1,1,1")
    print("PASS  test_firmware_log_lines        50 samples + 6 '#'/'!' lines (CRLF, one first after "
          "connecting): log counted and kept, 0 samples lost, only the real garbage line is bad")


if __name__ == "__main__":
    test_parse_line()
    test_line_assembler()
    test_link_over_pty()
    test_link_open_failure_is_reported()
    test_firmware_log_lines_are_skipped()
    print("\nAll STM32 link checks passed.")
