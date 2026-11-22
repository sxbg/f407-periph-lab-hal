"""Full-output capture (no early break) for loop commands + failing tests."""
import sys
import time
import serial

PORT, BAUD = "COM18", 115200

def run_full(cmd, timeout):
    s = serial.Serial(PORT, BAUD, timeout=0.3)
    s.dtr = False; s.rts = False
    time.sleep(0.2)
    s.reset_input_buffer()
    s.write(b"\r\n"); time.sleep(0.3)
    s.reset_input_buffer()
    for _ch in cmd:
        s.write(_ch.encode("ascii"))
        time.sleep(0.008)  # pace: fast writes dropped the last char
    s.write(b"\r\n")
    buf = ""
    deadline = time.time() + timeout
    while time.time() < deadline:
        c = s.read(512).decode("utf-8", errors="replace")
        if c:
            buf += c
    s.close()
    print(f"===== {cmd} =====")
    print(buf.strip())
    print()

for a in sys.argv[1:]:
    cmd, t = a.split(":")
    run_full(cmd, int(t))
