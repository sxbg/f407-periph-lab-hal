"""Automated UART menu test runner for f407-periph-lab (phase-1: non-interactive).

Usage: python menu_autotest.py [cmd1 cmd2 ...]
       (no args = run the default non-interactive suite)
"""
import sys
import time
import serial

PORT, BAUD = "COM18", 115200
MENU_TIMEOUT = 2.0

# command -> per-test timeout seconds (generous, tests self-bound)
SUITE = [
    ("uart", 5), ("gpio", 5), ("tim_pwm", 5), ("tim_basic", 5),
    ("tim_ic", 4), ("tim_enc", 4), ("tim_adv", 5), ("can_lb", 6),
    ("crc_rng", 5), ("rtc", 9), ("wdg", 5), ("pwr", 4),
    ("flash_iap", 9), ("i2c_ee", 6), ("spi_flash", 8), ("sdio_tf", 6),
    ("adc", 5), ("dac", 5), ("dma", 6),
]


def open_port():
    s = serial.Serial(PORT, BAUD, timeout=0.3)
    s.dtr = False   # avoid accidental reset pulse on some DAP VCPs
    s.rts = False
    time.sleep(0.2)
    return s


def drain(s, t=0.5):
    s.timeout = t
    out = b""
    while True:
        chunk = s.read(256)
        if not chunk:
            break
        out += chunk
    return out.decode("utf-8", errors="replace")


def run_cmd(s, cmd, timeout):
    s.reset_input_buffer()
    for _ch in cmd:
        s.write(_ch.encode("ascii"))
        time.sleep(0.008)  # pace: fast writes dropped the last char
    s.write(b"\r\n")
    s.timeout = 0.3
    buf = ""
    deadline = time.time() + timeout
    verdict = None
    while time.time() < deadline:
        chunk = s.read(512).decode("utf-8", errors="replace")
        if chunk:
            buf += chunk
            for token in ("PASS", "FAIL"):
                if token in buf.replace("PASS uart", ""):
                    # naive scan refined below by explicit search of final line
                    pass
            if "\nPASS" in buf or buf.strip().endswith("PASS") or "PASS " in buf.splitlines()[-1] if buf.splitlines() else False:
                pass
            # robust: any line containing PASS/FAIL for this command
            for line in buf.splitlines():
                ls = line.strip()
                if ls.startswith(f"PASS {cmd}") or ls.startswith(f"FAIL {cmd}"):
                    verdict = ls
        else:
            # small idle gap; keep reading until verdict or deadline
            pass
        if verdict:
            break
    return buf, verdict


def main():
    cmds = sys.argv[1:]
    suite = [(c, dict(SUITE).get(c, 8)) for c in cmds] if cmds else SUITE
    s = open_port()
    wake = drain(s, 1.0)
    # poke the menu: empty line -> help is NOT triggered; just send one CR
    s.write(b"\r\n")
    time.sleep(0.3)
    drain(s, 0.5)

    results = []
    for cmd, tmo in suite:
        buf, verdict = run_cmd(s, cmd, tmo)
        status = verdict if verdict else f"NO-VERDICT (tail: {buf.strip()[-90:]!r})"
        results.append((cmd, status))
        print(f"[{cmd:10s}] {status}")
    print("\n==== SUMMARY ====")
    npass = sum(1 for c, st in results if st.startswith("PASS"))
    nfail = sum(1 for c, st in results if st.startswith("FAIL"))
    print(f"PASS {npass} / FAIL {nfail} / OTHER {len(results)-npass-nfail}")
    s.close()


if __name__ == "__main__":
    main()
