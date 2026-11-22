"""Automated UART menu test runner for f407-periph-lab-hal (W6 full suite).

Usage: python menu_autotest.py [cmd1 cmd2 ...]
       (no args = run the full 22-item non-interactive suite)

Wire-dependent items (dac_adc_loop needs PA4->PC0, pwm_ic_loop needs
PA8->PB3) run automatically -- keep the jumpers connected.
`exti` runs the idle path by default (PASS); the pressed path (>= 3
KEY2 presses) is the W6 manual check.

KNOWN: expected deviations recorded in docs/test-report.md -- counted
separately from FAIL (watchdog disabled for debugging: user decision).
"""
import sys
import time
import serial

PORT, BAUD = "COM18", 115200

# command -> per-test timeout seconds (generous, tests self-bound)
SUITE = [
    ("uart", 8), ("gpio", 8), ("tim_pwm", 8), ("tim_basic", 12),
    ("tim_ic", 6), ("tim_enc", 6), ("tim_adv", 5), ("can_lb", 6),
    ("crc_rng", 5), ("rtc", 10), ("wdg", 5), ("pwr", 4),
    ("flash_iap", 10), ("i2c_ee", 6), ("spi_flash", 10), ("sdio_tf", 8),
    ("adc", 6), ("dac", 5), ("dma", 15), ("oled", 15),
    ("dac_adc_loop", 15), ("pwm_ic_loop", 16),
]

# expected deviations -- not FAILs (see docs/test-report.md KNOWN section)
KNOWN = {
    "wdg": "watchdog fully disabled for debugging (user decision)",
}


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


def wait_prompt(s, timeout=15):
    """Read until the menu> prompt appears (banner/help fully drained).
    Typing while a HAL_UART_Transmit print is still unwinding loses
    characters to the RX overrun -- this sync is the pacing fix."""
    s.timeout = 0.2
    buf = b""
    t0 = time.time()
    while time.time() - t0 < timeout:
        chunk = s.read(256)
        if chunk:
            buf += chunk
            if b"menu> " in buf[-8:]:
                return buf.decode("utf-8", errors="replace")
    return buf.decode("utf-8", errors="replace")


def run_cmd(s, cmd, timeout):
    wait_prompt(s)  # sync before committing a command
    s.reset_input_buffer()
    s.timeout = 0.1
    while s.read(256):
        pass  # drain any stale echo
    for _ch in cmd:
        s.write(_ch.encode("ascii"))
        time.sleep(0.015)
    s.write(b"\r\n")
    s.timeout = 0.3
    buf = ""
    verdict = None
    deadline = time.time() + timeout
    while time.time() < deadline:
        chunk = s.read(512).decode("utf-8", errors="replace")
        if chunk:
            buf += chunk
            for line in buf.splitlines():
                ls = line.strip()
                if ls.startswith("PASS %s" % cmd) or ls.startswith("FAIL %s" % cmd):
                    verdict = ls
        if verdict:
            break
    wait_prompt(s)  # re-sync for the next command
    return buf, verdict


def main():
    cmds = sys.argv[1:]
    suite = [(c, dict(SUITE).get(c, 8)) for c in cmds] if cmds else SUITE
    s = open_port()
    wait_prompt(s)  # first sync waits out the banner + help print
    drain(s, 0.2)

    results = []
    for cmd, tmo in suite:
        buf, verdict = run_cmd(s, cmd, tmo)
        if verdict is None:
            status = "NO-VERDICT (tail: %r)" % buf.strip()[-90:]
        elif cmd in KNOWN and verdict.startswith("FAIL"):
            status = "KNOWN " + verdict + "  -- " + KNOWN[cmd]
        else:
            status = verdict
        results.append((cmd, status))
        print("[%10s] %s" % (cmd, status))

    print("")
    print("==== SUMMARY ====")
    npass = sum(1 for c, st in results if st.startswith("PASS"))
    nknown = sum(1 for c, st in results if st.startswith("KNOWN"))
    nfail = sum(1 for c, st in results if st.startswith("FAIL"))
    nother = len(results) - npass - nknown - nfail
    print("PASS %d / KNOWN %d / FAIL %d / OTHER %d"
          % (npass, nknown, nfail, nother))
    print("")
    print("==== MARKDOWN (paste into docs/test-report.md) ====")
    for cmd, st in results:
        mark = st.split()[0]
        note = st[len(mark):].strip().replace("|", "/")
        print("| %s | %s | %s |" % (cmd, mark, note))
    s.close()


if __name__ == "__main__":
    main()
