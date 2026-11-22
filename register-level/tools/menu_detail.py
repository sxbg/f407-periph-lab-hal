"""Full-output capture for specific menu commands (debug aid)."""
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
    s.write((cmd + "\r\n").encode("ascii"))
    buf = ""
    deadline = time.time() + timeout
    while time.time() < deadline:
        c = s.read(512).decode("utf-8", errors="replace")
        if c: buf += c
        lines = buf.splitlines()
        if any(l.strip().startswith(("PASS", "FAIL")) for l in lines[-3:]):
            break
    s.close()
    print(f"===== {cmd} =====")
    print(buf)

for cmd, t in [(a, int(b)) for a, b in (x.split(":") for x in sys.argv[1:])]:
    run_full(cmd, t)
