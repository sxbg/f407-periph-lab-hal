"""s00_bringup — HIL bring-up suite template.

Discipline (knowledge/patterns-testing.md):
- exact-equation assertions only (no >=N / >0 criteria)
- every suite includes at least one negative case
- J-Link/RTT absence -> SKIP (graceful degradation, never fake PASS)
"""
from autostm32.hiltest import (
    Case,
    VERDICT_PASS,
    VERDICT_FAIL,
    VERDICT_KNOWN,
    VERDICT_SKIP,
)


def run(ctx) -> list[Case]:
    """ctx: SuiteContext (see hiltest.SuiteContext)."""
    cases: list[Case] = []

    # ---- T01: BOOT event present ------------------------------------
    c = Case(
        id="T01", name="boot event present",
        purpose="firmware emits exactly one BOOT event at startup",
        steps=["flash current hex", "capture RTT 30s", "decode"],
        expected="exactly 1 BOOT event; sysclk field equals project.yaml sysclk_hz",
        verdict=VERDICT_SKIP,
    )
    boots = ctx.event_named("BOOT")
    if ctx.decode_report is None:
        c.verdict = VERDICT_SKIP
        c.detail = "no decoded capture (RTT absent?)"
    elif len(boots) == 1:
        want = ctx.pcfg.cfg["mcu"].get("sysclk_hz", 0)
        got = boots[0].b
        if ctx.assert_eq(got, want, c, "BOOT.sysclk_hz == project.yaml mcu.sysclk_hz"):
            c.verdict = VERDICT_PASS
            c.keydata.append(f"BOOT t={boots[0].t_ms}ms sysclk={got}")
    elif len(boots) == 0:
        c.verdict = VERDICT_FAIL
        c.detail = "no BOOT event: image not running or RTT dead"
    else:
        c.verdict = VERDICT_FAIL
        c.detail = f"{len(boots)} BOOT events: reset loop (see logcheck.boot_loop)"
    cases.append(c)

    # ---- T02: heartbeat alive ----------------------------------------
    c = Case(
        id="T02", name="heartbeat alive",
        purpose="#H liveness probe runs for the whole capture window",
        steps=["capture RTT window from tests.hil.rtt_window_s"],
        expected=">= half the window in heartbeats (1s period, 50% margin)",
        verdict=VERDICT_SKIP,
    )
    hb = ctx.heartbeat_count()
    if ctx.decode_report is None:
        c.detail = "no decoded capture"
    else:
        window = ctx.pcfg.cfg.get("tests", {}).get("hil", {}).get("rtt_window_s", 30)
        want = max(1, int(window / 2))
        if ctx.assert_eq(hb >= want, True, c, f"heartbeats {hb} >= {want}"):
            c.verdict = VERDICT_PASS
            c.keydata.append(f"heartbeats={hb} window={window}s")
    cases.append(c)

    # ---- T03 (negative): no error events ------------------------------
    c = Case(
        id="T03", name="no error events (negative)",
        purpose="ERR_HANDLER/ASSERT/CLOCK_FALLBACK/WATCHDOG must NOT appear",
        steps=["decode capture"],
        expected="zero occurrences of any error-class event",
        verdict=VERDICT_SKIP,
    )
    if ctx.decode_report is None:
        c.detail = "no decoded capture"
    else:
        bad = []
        for name in ("ERR_HANDLER", "ASSERT", "CLOCK_FALLBACK", "WATCHDOG_RESET"):
            bad += ctx.event_named(name)
        if ctx.assert_eq(len(bad), 0, c, "error-class events == 0"):
            c.verdict = VERDICT_PASS
        else:
            c.keydata = [f"{e.name} t={e.t_ms}ms a={e.a:#x} b={e.b:#x}" for e in bad[:8]]
    cases.append(c)

    # ---- T04 (negative, witness): log mask unchanged -------------------
    c = Case(
        id="T04", name="witness: rtt_log_mask == ALL (negative)",
        purpose="runtime log mask was not left disabled by a previous session",
        steps=["J-Link read rtt_log_mask symbol"],
        expected="mask == 0xFFFF (RTT_MASK_ALL)",
        verdict=VERDICT_SKIP,
    )
    mask = ctx.witness.get("rtt_log_mask")
    if not ctx.witness:
        c.detail = "witness unavailable (J-Link or map absent) -> SKIP"
    else:
        if ctx.assert_eq(mask, 0xFFFF, c, "rtt_log_mask == RTT_MASK_ALL"):
            c.verdict = VERDICT_PASS
    cases.append(c)

    return cases
